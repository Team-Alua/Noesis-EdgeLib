from inc_noesis import *
import noesis
import rapi
import struct
import math
from EdgeData20 import *
from GravityRush_common import *
import json
import warnings
warnings.simplefilter("ignore", DeprecationWarning)

# skel_path = "kit01_backflip_00.skel"
# anim_path = "kit01_backflip_00_source.anim"
#
# def main():
#     skeleton = None
#     if skel_path != None:
#         skel_file = open(skel_path, mode='rb')
#         skel_data = skel_file.read()
#         pSkel = EdgeAnimationSkeleton(skel_data)
#         skeleton = ExtractSkeleton(pSkel)
#         printSkelLog(pSkel, skeleton, skel_path)
#
#     if anim_path != None:
#         anim_file = open(anim_path, mode='rb')
#         anim_data = anim_file.read()
#         pAnim = EdgeAnimAnimation(anim_data)
#         compressssed = ReadAnimation(pAnim)
#         animation = DecompressAnimation(compressssed, skeleton)
#         printAnimLog(pAnim, compressssed, animation, anim_path)

# Skeleton
def ExtractSkeleton(pSkel):
    # check tag + determine endian-ness
    endianSwap = pSkel.bigEndian

    skeleton = Skeleton()
    skeleton.m_locoJointIndex = pSkel.locomotionJointIndex
    skeleton.m_numJoints = pSkel.numJoints

    #parent indices & scale compensate flags
    jointLinkage = 0x40
    numSimdElements = pSkel.numJointLinkages
    idxList = struct.unpack(('<', '>')[endianSwap] + 'HH' * numSimdElements, pSkel.data[jointLinkage : jointLinkage + numSimdElements * 4])
    skeleton.m_parentIndices = [-1] * skeleton.m_numJoints
    skeleton.m_scaleCompensateFlags = [False] * skeleton.m_numJoints
    for jointIndex in range(numSimdElements):
        idxJoint = idxList[jointIndex * 2]
        idxParent = idxList[jointIndex * 2 + 1]

        parent = (idxParent & 0x7fff, -1)[(idxParent & 0x7fff) >= 0x4000]
        skeleton.m_parentIndices[idxJoint] = parent

        compensateParent = (idxParent & 0x8000) == 0x8000
        skeleton.m_scaleCompensateFlags[idxJoint] = compensateParent

    # Here suppose to be something with pJointLinkageWidth, but main.cpp always set pJointLinkageWidth to 0 so it will always ignored this part

    # base pose
    pJoints = pSkel.offsetBasePose
    for i in range(skeleton.m_numJoints):
        joint = Joint(pSkel.data[pJoints : pJoints + 48], pSkel.bigEndian)
        skeleton.m_basePose.append(joint)
        pJoints += 48

    #  joint hash names
    pJointNames = pSkel.offsetJointNameHashArray
    skeleton.m_jointNameHashes = struct.unpack(('<', '>')[endianSwap] + 'I' * skeleton.m_numJoints, pSkel.data[pJointNames : pJointNames + skeleton.m_numJoints * 4])

    # user channel info
    idBit = EdgeAnimUserChannelFlags.EDGE_ANIM_USER_CHANNEL_FLAG_CPT_SHIFT
    idMask = EdgeAnimUserChannelFlags.EDGE_ANIM_USER_CHANNEL_FLAG_CPT_BIT0 | EdgeAnimUserChannelFlags.EDGE_ANIM_USER_CHANNEL_FLAG_CPT_BIT1
    skeleton.m_numUserChannels = pSkel.numUserChannels
    pUserChannelNodeNames = pSkel.offsetUserChannelNodeNameHashArray
    nodeHashList = struct.unpack(('<', '>')[endianSwap] + 'I' * skeleton.m_numUserChannels, pSkel.data[pUserChannelNodeNames : pUserChannelNodeNames + skeleton.m_numUserChannels * 4])
    pUserChannelNames = pSkel.offsetUserChannelNameHashArray
    chanHashList = struct.unpack(('<', '>')[endianSwap] + 'I' * skeleton.m_numUserChannels, pSkel.data[pUserChannelNames: pUserChannelNames + skeleton.m_numUserChannels * 4])
    pUserChannelFlags = pSkel.offsetUserChannelFlagsArray
    flagList = struct.unpack(('<', '>')[endianSwap] + 'B' * skeleton.m_numUserChannels, pSkel.data[pUserChannelFlags: pUserChannelFlags + skeleton.m_numUserChannels])
    for i in range(skeleton.m_numUserChannels):
        info = UserChannelInfo(nodeHashList[i], chanHashList[i], flagList[i] & (~idMask), (flagList[i] & idMask) >> idBit)
        skeleton.m_userChannelInfoArray.append(info)

    # custom data
    sizeCustomData = pSkel.sizeCustomData
    pCustomData = pSkel.offsetCustomData
    skeleton.m_customData = pSkel.data[pCustomData : pCustomData + sizeCustomData]

    return skeleton

def printSkelLog(pSkel, skeleton, skel_path = None, printHierachy = False):
    print("EdgeAnimSkeleton")
    if skel_path != None:
        print("File:               %s" % skel_path)
    print("Endianness:         %s" % ("LittleEndian", "BigEndian")[pSkel.bigEndian])
    print("Size:               %d bytes" % pSkel.length)
    print("Number of Joints:   %d" % skeleton.m_numJoints )
    print("Number of Channels: %d" % skeleton.m_numUserChannels)
    #print("Joint SIMD Width:   %d" % jointLinkageWidth)
    print("Locomotion Joint:   %d" % skeleton.m_locoJointIndex )

    if skeleton.m_numJoints:
        print("Joint Hierachy:")
        for jointIndex in range(skeleton.m_numJoints):
            if (printHierachy == False) or (skeleton.m_parentIndices[jointIndex] < 0) :
                joint = skeleton.m_basePose[jointIndex]
                LogSkeletonJoint(skeleton, joint, jointIndex, '', printHierachy)

        print()

    if skeleton.m_numUserChannels:
        print("User Channels:")
        for channelIndex in range(skeleton.m_numUserChannels):
            userChannel = skeleton.m_userChannelInfoArray[channelIndex]
            print(" Channel[%d]" % channelIndex)
            print(" Name:      %s, %s" % (unHash(userChannel.m_nodeNameHash), unHash(userChannel.m_channelNameHash)))
            print(" Component: %d" % userChannel.m_componentIndex)
            print(" Flags:     %s %s" % (hex(userChannel.m_flags), strEdgeAnimUserChannelFlags(userChannel.m_flags)))
        print()

def LogSkeletonJoint(skeleton, joint, jointIndex, indent, printHierachy = False):
    print("%s Joint[%d]" % (indent, jointIndex))
    print("%s  Name:   %s" % (indent, unHash(skeleton.m_jointNameHashes[jointIndex])))
    print("%s  Parent: %d%s" % (indent, skeleton.m_parentIndices[jointIndex], ("", " (ScaleCompensated)")[skeleton.m_scaleCompensateFlags[ jointIndex ]]))
    print("%s  Quat:   %f, %f, %f, %f" % (indent, joint.m_rotation[0], joint.m_rotation[1], joint.m_rotation[2], joint.m_rotation[3]))
    print("%s  Trans:  %f, %f, %f" % (indent, joint.m_translation[0], joint.m_translation[1], joint.m_translation[2]))
    print("%s  Scale:  %f, %f, %f" % (indent, joint.m_scale[0], joint.m_scale[1], joint.m_scale[2]))

    if(printHierachy):
        for childIndex in range(skeleton.m_numJoints):
            if skeleton.m_parentIndice[childIndex] == jointIndex:
                child = skeleton.m_basePose[childIndex]
                LogSkeletonJoint(skeleton, childIndex, indent + "  ", printHierachy)

# Animation
def ReadAnimation(pAnim):
    animation = CompressedAnimation()
    # check tag + determine endian-ness
    endianSwap = pAnim.bigEndian

    # check joint count is consistent with the bind skeleton
    animation.m_numJoints = pAnim.numJoints
    
    # check user channel count is consistent with the bind skeleton
    animation.m_numUserChannels = pAnim.numUserChannels

    # Extract header details
    animation.m_duration = pAnim.duration
    animation.m_sampleFrequency = pAnim.sampleFrequency
    animation.m_numFrames = pAnim.numFrames
    animation.m_numFrameSets = pAnim.numFrameSets
    animation.m_evalBufferSizeRequired = pAnim.evalBufferSizeRequired
    animation.m_numConstRChannels = pAnim.numConstRChannels
    animation.m_numConstTChannels = pAnim.numConstTChannels
    animation.m_numConstSChannels = pAnim.numConstSChannels
    animation.m_numConstUChannels = pAnim.numConstUserChannels
    animation.m_numAnimRChannels = pAnim.numAnimRChannels
    animation.m_numAnimTChannels = pAnim.numAnimTChannels
    animation.m_numAnimSChannels = pAnim.numAnimSChannels
    animation.m_numAnimUChannels = pAnim.numAnimUserChannels
    flags = pAnim.flags

    # joint + user + weights
    animation.m_enableWeights = False
    # animation.m_jointWeights.clear()
    # animation.m_userChannelWeights.clear()

    sizeJointsWeightArray = pAnim.sizeJointsWeightArray
    sizeUserChannelWeightArray = pAnim.sizeUserChannelWeightArray
    sizeWeightArray = sizeJointsWeightArray + sizeUserChannelWeightArray
    if sizeWeightArray > 0:
        animation.m_enableWeights = True
        offsetJointsWeightArray = pAnim.offsetJointsWeightArray
        if offsetJointsWeightArray == 0:
            print("Invalid edge animation binary missing joints weighting data")
            exit
        animation.m_jointWeights = struct.unpack('B'*pAnim.numJoints, pAnim.data[offsetJointsWeightArray: offsetJointsWeightArray + pAnim.numJoints])

        offsetUserChannelWeightArray = pAnim.offsetUserChannelWeightArray
        if offsetUserChannelWeightArray == 0:
            print("Invalid edge animation binary missing user channels weighting data")
            exit
        animation.m_userChannelWeights = struct.unpack(('<', '>')[endianSwap]  + 'B' * pAnim.numUserChannels, pAnim.data[offsetUserChannelWeightArray: offsetUserChannelWeightArray + pAnim.numUserChannels])

    # locomotion delta - Untested, might have issue
    animation.m_enableLocoDelta = False
    if pAnim.offsetLocomotionDelta:
        animation.m_enableLocoDelta = True
        animation.m_locoDeltaQuat = struct.unpack(('<', '>')[endianSwap] + '4f' * pAnim.numUserChannels, pAnim.data[pAnim.offsetLocomotionDelta : pAnim.offsetLocomotionDelta + 16])
        animation.m_locoDeltaTrans = struct.unpack(('<', '>')[endianSwap] + '4f' * pAnim.numUserChannels, pAnim.data[pAnim.offsetLocomotionDelta + 16 : pAnim.offsetLocomotionDelta + 32])

    # custom data
    animation.m_customData = pAnim.data[pAnim.offsetCustomData: pAnim.offsetCustomData + pAnim.sizeCustomData]

    # channel tables
    constRTable = 0x60
    constTTable = constRTable + EDGE_ALIGN(animation.m_numConstRChannels, 8, 2)
    constSTable = constTTable + EDGE_ALIGN(animation.m_numConstTChannels, 4, 2)
    constUTable = constSTable + EDGE_ALIGN(animation.m_numConstSChannels, 4, 2)
    animRTable = constUTable + EDGE_ALIGN(animation.m_numConstUChannels, 4, 2)
    animTTable = animRTable + EDGE_ALIGN(animation.m_numAnimRChannels, 4, 2)
    animSTable = animTTable + EDGE_ALIGN(animation.m_numAnimTChannels, 4, 2)
    animUTable = animSTable + EDGE_ALIGN(animation.m_numAnimSChannels, 4, 2)

    # extract compression modes
    animation.m_compressionInfo = CompressionInfo()
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_R:
        animation.m_compressionInfo.m_compressionTypeRotation = EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED
    elif flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_RAW_R:
        animation.m_compressionInfo.m_compressionTypeRotation = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE
    else:
        animation.m_compressionInfo.m_compressionTypeRotation = EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_T:
       animation.m_compressionInfo.m_compressionTypeTranslation = EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED
    else:
        animation.m_compressionInfo.m_compressionTypeTranslation = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_S:
        animation.m_compressionInfo.m_compressionTypeScale = EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED
    else:
        animation.m_compressionInfo.m_compressionTypeScale = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_U:
        animation.m_compressionInfo.m_compressionTypeUser = EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED
    else:
        animation.m_compressionInfo.m_compressionTypeUser = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE

    # extract constant channel tables
    animation.m_constRChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numConstRChannels, pAnim.data[constRTable : constRTable + animation.m_numConstRChannels * 2])
    animation.m_constTChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numConstTChannels, pAnim.data[constTTable : constTTable + animation.m_numConstTChannels * 2])
    animation.m_constSChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numConstSChannels, pAnim.data[constSTable : constSTable + animation.m_numConstSChannels * 2])
    animation.m_constUChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numConstUChannels, pAnim.data[constUTable : constUTable + animation.m_numConstUChannels * 2])

    # extract animated channel tables
    animation.m_animRChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numAnimRChannels, pAnim.data[animRTable : animRTable + animation.m_numAnimRChannels * 2])
    animation.m_animTChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numAnimTChannels, pAnim.data[animTTable : animTTable + animation.m_numAnimTChannels * 2])
    animation.m_animSChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numAnimSChannels, pAnim.data[animSTable : animSTable + animation.m_numAnimSChannels * 2])
    animation.m_animUChannels = struct.unpack(('<', '>')[endianSwap] + 'H' * animation.m_numAnimUChannels, pAnim.data[animUTable : animUTable + animation.m_numAnimUChannels * 2])

    # extract packing specs
    pPackingSpecs = pAnim.offsetPackingSpecs
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_R:
        for i in range(animation.m_numAnimRChannels + 1):
            spec = KeyframePackingSpec(struct.unpack(('<', '>')[endianSwap] + 'i', pAnim.data[pPackingSpecs : pPackingSpecs + 4])[0])
            animation.m_packingSpecsR.append(spec)
            pPackingSpecs += 4
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_T:
        for i in range(animation.m_numAnimTChannels + 1):
            spec = KeyframePackingSpec(struct.unpack(('<', '>')[endianSwap] + 'i', pAnim.data[pPackingSpecs: pPackingSpecs + 4])[0])
            animation.m_packingSpecsT.append(spec)
            pPackingSpecs += 4
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_S:
        for i in range(animation.m_numAnimSChannels + 1):
            spec = KeyframePackingSpec(struct.unpack(('<', '>')[endianSwap] + 'i', pAnim.data[pPackingSpecs: pPackingSpecs + 4])[0])
            animation.m_packingSpecsS.append(spec)
            pPackingSpecs += 4
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_U:
        for i in range(animation.m_numAnimUChannels + 1):
            spec = KeyframePackingSpec(struct.unpack(('<', '>')[endianSwap] + 'i', pAnim.data[pPackingSpecs: pPackingSpecs + 4])[0])
            animation.m_packingSpecsU.append(spec)
            pPackingSpecs += 4

    # extract constant data
    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_R:
        spec = animation.m_packingSpecsR[0]
        bit8Size = EDGE_ALIGN(animation.m_numConstRChannels * spec.GetNumBits(), 8, 1)
        byteSize = bit8Size >> 3

        pConstData = pAnim.offsetConstRData
        animation.m_constRBitpacked = pAnim.data[pConstData: pConstData + byteSize]

    elif flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_RAW_R:
        pConstData = pAnim.offsetConstRData
        for channel in range(animation.m_numConstRChannels):
            animation.m_constRRaw.append(struct.unpack(('<', '>')[endianSwap] + 'ffff', pAnim.data[pConstData : pConstData + 16]))
            pConstData += 16
    else:
        pConstData = pAnim.offsetConstRData
        for channel in range(animation.m_numConstRChannels):
            data = pAnim.data[pConstData : pConstData + 6]
            animation.m_constRSmallest3.append(int.from_bytes(data, "big"))
            pConstData += 6

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_T:
        spec = animation.m_packingSpecsT[0]
        bit8Size = EDGE_ALIGN(animation.m_numConstTChannels * spec.GetNumBits(), 8, 1)
        byteSize = bit8Size >> 3

        pConstData = pAnim.offsetConstTData
        animation.m_constTBitpacked = pAnim.data[pConstData: pConstData + byteSize]
    else:
        pConstData = pAnim.offsetConstTData
        for channel in range(animation.m_numConstTChannels):
            animation.m_constTRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pConstData : pConstData + 12])))
            animation.m_constTRaw[-1].append(1.0)
            pConstData += 12

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_S:
        spec = animation.m_packingSpecsS[0]
        bit8Size = EDGE_ALIGN(animation.m_numConstSChannels * spec.GetNumBits(), 8, 1)
        byteSize = bit8Size >> 3

        pConstData = pAnim.offsetConstSData
        animation.m_constSBitpacked = pAnim.data[pConstData: pConstData + byteSize]
    else:
        pConstData = pAnim.offsetConstSData
        for channel in range(animation.m_numConstSChannels):
            animation.m_constSRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pConstData : pConstData + 12])))
            animation.m_constSRaw[-1].append(1.0)
            pConstData += 12

    if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_U:
        spec = animation.m_packingSpecsU[0]
        bit8Size = EDGE_ALIGN(animation.m_numConstUChannels * spec.GetNumBits(), 8, 1)
        byteSize = bit8Size >> 3

        pConstData = pAnim.offsetConstUData
        animation.m_constUBitpacked = pAnim.data[pConstData: pConstData + byteSize]
    else:
        pConstData = pAnim.offsetConstUserData
        for channel in range(animation.m_numConstUChannels):
            animation.m_constURaw.append(struct.unpack(('<', '>')[endianSwap] + 'f', pAnim.data[pConstData : pConstData + 4])[0])
            pConstData += 4

    #extract frame-sets
    pFrameSetDmaArray = []
    pFrameSetInfoArray = []
    for frameSetIndex in range(animation.m_numFrameSets):
        #print("Frame Set #%i" % frameSetIndex)
        pFrameSetDmaArray.append(EdgeDmaListElement(pAnim.data[pAnim.offsetFrameSetDmaArray + frameSetIndex * 8: pAnim.offsetFrameSetDmaArray + frameSetIndex * 8 + 8], endianSwap))
        pFrameSetInfoArray.append(EdgeAnimFrameSetInfo(pAnim.data[pAnim.offsetFrameSetInfoArray + frameSetIndex * 4 : pAnim.offsetFrameSetInfoArray + frameSetIndex * 4 + 4], endianSwap))
        frameSet = CompressedFrameSet()
        frameSet.m_baseFrame = pFrameSetInfoArray[frameSetIndex].baseFrame
        frameSet.m_numIntraFrames = pFrameSetInfoArray[frameSetIndex].numIntraFrames

        frameSetData = pFrameSetDmaArray[frameSetIndex].eal

        sizeInitialRData, \
        sizeInitialTData, \
        sizeInitialSData, \
        sizeInitialUData, \
        sizeIntraRData, \
        sizeIntraTData, \
        sizeIntraSData, \
        sizeIntraUData \
        = struct.unpack(('<', '>')[endianSwap] + "IIIIIIII", pAnim.data[frameSetData : frameSetData + 32])

        sizeIntraBits = animation.m_numAnimRChannels
        sizeIntraBits += animation.m_numAnimTChannels
        sizeIntraBits += animation.m_numAnimSChannels
        sizeIntraBits += animation.m_numAnimUChannels
        sizeIntraBits *= frameSet.m_numIntraFrames
        sizeIntraBits = EDGE_ALIGN(sizeIntraBits, 8, 1)
        sizeIntraBits >>= 3

        initialRAdr = frameSetData + 8 * 4
        initialTAdr = initialRAdr + sizeInitialRData
        initialSAdr = initialTAdr + sizeInitialTData
        initialUAdr = initialSAdr + sizeInitialSData
        intraBitsAdr = initialUAdr + sizeInitialUData
        intraRAdr = intraBitsAdr + sizeIntraBits
        intraTAdr = intraRAdr + sizeIntraRData
        intraSAdr = intraTAdr + sizeIntraTData
        intraUAdr = EDGE_ALIGN(intraSAdr + sizeIntraSData, 4, 1)

        #initial joint data
        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_R:
            frameSet.m_initialRBitpacked = pAnim.data[initialRAdr : initialRAdr + sizeInitialRData]

        elif flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_RAW_R:
            pAnimData = initialRAdr
            countAnimData = sizeInitialRData // 16
            for keyFrame in range(countAnimData):
                frameSet.m_initialRRaw.append(struct.unpack(('<', '>')[endianSwap] + 'ffff', pAnim.data[pAnimData: pAnimData + 16]))
                pAnimData += 16
        else:
            pAnimData = initialRAdr
            countAnimData = sizeInitialRData // 6
            for channel in range(countAnimData):
                data = pAnim.data[pAnimData: pAnimData + 6]
                frameSet.m_initialRSmallest3.append(int.from_bytes(data, "big"))
                pAnimData += 6

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_T:
            frameSet.m_initialTBitpacked = pAnim.data[initialTAdr : initialTAdr + sizeInitialTData]
        else:
            pAnimData = initialTAdr
            countAnimData = sizeInitialTData // 12
            for keyFrame in range(countAnimData):
                frameSet.m_initialTRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pAnimData: pAnimData + 12])))
                frameSet.m_initialTRaw[-1].append(1.0)
                pAnimData += 12

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_S:
            frameSet.m_initialSBitpacked = pAnim.data[initialSAdr : initialSAdr + sizeInitialSData]
        else:
            pAnimData = initialSAdr
            countAnimData = sizeInitialSData // 12
            for keyFrame in range(countAnimData):
                frameSet.m_initialSRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pAnimData: pAnimData + 12])))
                frameSet.m_initialSRaw[-1].append(1.0)
                pAnimData += 12

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_U:
            frameSet.m_initialUBitpacked = pAnim.data[initialUAdr : initialUAdr + sizeInitialUData]
        else:
            pAnimData = initialUAdr
            countAnimData = sizeInitialUData // 4
            for keyFrame in range(countAnimData):
                frameSet.m_initialURaw.append(struct.unpack(('<', '>')[endianSwap] + 'f', pAnim.data[pAnimData: pAnimData + 4]))
                pAnimData += 4

        # intra frame bit stream
        frameSet.m_intraBits = pAnim.data[intraBitsAdr: intraBitsAdr + sizeIntraBits]

        # intra joint data
        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_R:
            frameSet.m_intraRBitpacked = pAnim.data[intraRAdr : intraRAdr + sizeIntraRData]
        elif flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_RAW_R:
            pAnimData = intraRAdr
            countAnimData = sizeIntraRData // 16
            for keyFrame in range(countAnimData):
                frameSet.m_intraRRaw.append(struct.unpack(('<', '>')[endianSwap] + 'ffff', pAnim.data[pAnimData : pAnimData + 16]))
                pAnimData += 16
        else:
            pAnimData = intraRAdr
            countAnimData = sizeIntraRData // 6
            for channel in range(countAnimData):
                data = pAnim.data[pAnimData: pAnimData + 6]
                frameSet.m_intraRSmallest3.append(int.from_bytes(data, "big"))
                pAnimData += 6

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_T:
            frameSet.m_intraTBitpacked = pAnim.data[intraTAdr : intraTAdr + sizeIntraTData]
        else:
            pAnimData = intraTAdr
            countAnimData = sizeIntraTData // 12
            for keyFrame in range(countAnimData):
                frameSet.m_intraTRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pAnimData : pAnimData + 12])))
                frameSet.m_intraTRaw[-1].append(1.0)
                pAnimData += 12

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_S:
            frameSet.m_intraSBitpacked = pAnim.data[intraSAdr : intraSAdr + sizeIntraSData]
        else:
            pAnimData = intraSAdr
            countAnimData = sizeIntraSData // 12
            for keyFrame in range(countAnimData):
                frameSet.m_intraSRaw.append(list(struct.unpack(('<', '>')[endianSwap] + 'fff', pAnim.data[pAnimData : pAnimData + 12])))
                frameSet.m_intraSRaw[-1].append(1.0)
                pAnimData += 12

        if flags & EdgeAnimAnimationEncodingFlags.EDGE_ANIM_FLAG_BIT_PACKED_U:
            frameSet.m_intraUBitpacked = pAnim.data[intraUAdr : intraUAdr + sizeIntraUData]
        else:
            pAnimData = intraUAdr
            countAnimData = sizeIntraUData // 4
            for keyFrame in range(countAnimData):
                frameSet.m_intraURaw.append(struct.unpack(('<', '>')[endianSwap] + 'f', pAnim.data[pAnimData: pAnimData + 4]))
                pAnimData += 4

        animation.m_frameSets.append(frameSet)

    return animation

def DecompressAnimation(compressed, bindSkeleton = None):
    animation = Animation()
    animation.m_startTime = 0.0
    animation.m_endTime = compressed.m_duration
    animation.m_period = 1.0 / compressed.m_sampleFrequency
    animation.m_numFrames = compressed.m_numFrames

    # locomotion
    animation.m_enableLocoDelta = compressed.m_enableLocoDelta
    animation.m_locoDeltaQuat = compressed.m_locoDeltaQuat
    animation.m_locoDeltaTrans = compressed.m_locoDeltaTrans

    # Partial weight
    animation.m_enableWeights = compressed.m_enableWeights
    animation.m_jointWeights = compressed.m_jointWeights
    animation.m_userChannelWeights = compressed.m_userChannelWeights

    # set animation detail to empty (equivalent to base pose)
    if bindSkeleton != None:
        for jointIndex in range(bindSkeleton.m_numJoints):
            joint = JointAnimation()
            joint.m_jointName = bindSkeleton.m_jointNameHashes[jointIndex]
            joint.m_jointWeight = 1.0
            animation.m_jointAnimations.append(joint)

        for channelIndex in range(bindSkeleton.m_numUserChannels):
            channel = UserChannelAnimation()
            channel.m_nodeName = bindSkeleton.m_userChannelInfoArray[channelIndex].m_nodeNameHash
            channel.m_channelName = bindSkeleton.m_userChannelInfoArray[channelIndex].m_channelNameHash
            channel.m_weight = 1.0
            animation.m_userChannelAnimations.append(channel)
    else:
        for jointIndex in range(compressed.m_numJoints):
            joint = JointAnimation()
            joint.m_jointName = jointIndex
            joint.m_jointWeight = 1.0
            animation.m_jointAnimations.append(joint)

        for channelIndex in range(compressed.m_numUserChannels):
            channel = UserChannelAnimation()
            channel.m_nodeName = channelIndex
            channel.m_channelName = channelIndex
            channel.m_weight = 1.0
            animation.m_userChannelAnimations.append(channel)

    # weights
    if compressed.m_enableWeights:
        for jointIndex in range(compressed.m_numJoints):
            joint = animation.m_jointAnimations[jointIndex]
            joint.m_jointWeight = compressed.m_jointWeights[jointIndex] / 255.0

        for channelIndex in range(compressed.m_numUserChannels):
            channel = animation.m_userChannelAnimations[channelIndex]
            channel.m_weight = compressed.m_userChannelWeights[channelIndex] / 255.0

    # decompression
    if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
        bitStream = NoeBitStream(reverseBitOrder(compressed.m_constRBitpacked))

    for i in range(len(compressed.m_constRChannels)):
        key = AnimationKeyframe()
        key.m_keyTime = animation.m_startTime
        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
            key.m_keyData = compressed.m_constRRaw[i]
        elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3:
            key.m_keyData = DecompressQuat(compressed.m_constRSmallest3[i])
        elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            key.m_keyData = BitDecompressQuat(bitStream, compressed.m_packingSpecsR[0])
        else:
            print("???? Unreachable")
            exit

        jointIndex = compressed.m_constRChannels[i]
        animation.m_jointAnimations[jointIndex].m_rotationAnimation.append(key)

    if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
        bitStream = NoeBitStream(reverseBitOrder(compressed.m_constTBitpacked))

    for i in range(len(compressed.m_constTChannels)):
        key = AnimationKeyframe()
        key.m_keyTime = animation.m_startTime
        if compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
            key.m_keyData = compressed.m_constTRaw[i]
        elif compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsT[0])
        else:
            print("???? Unreachable")
            exit

        jointIndex = compressed.m_constTChannels[i]
        animation.m_jointAnimations[jointIndex].m_translationAnimation.append(key)

    if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
        bitStream = NoeBitStream(reverseBitOrder(compressed.m_constSBitpacked))

    for i in range(len(compressed.m_constSChannels)):
        key = AnimationKeyframe()
        key.m_keyTime = animation.m_startTime
        if compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
            key.m_keyData = compressed.m_constSRaw[i]
        elif compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsS[0])
        else:
            print("???? Unreachable")
            exit

        jointIndex = compressed.m_constSChannels[i]
        animation.m_jointAnimations[jointIndex].m_scaleAnimation.append(key)

    if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
        bitStream = NoeBitStream(reverseBitOrder(compressed.m_constUBitpacked))

    for i in range(len(compressed.m_constUChannels)):
        key = AnimationKeyframe()
        key.m_keyTime = animation.m_startTime
        if compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
            key.m_keyData[0] = key.m_keyData[1] = key.m_keyData[2] = key.m_keyData[3] = compressed.m_constURaw[i]
        elif compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            key.m_keyData = BitDecompressFloat(bitStream, compressed.m_packingSpecsU[0]) #Not sure if that will works
        else:
            print("???? Unreachable")
            exit

        channelIndex = compressed.m_constUChannels[i]
        animation.m_userChannelAnimations[channelIndex].m_animation.append(key)

    for frameSetIndex in range(compressed.m_numFrameSets):
        frameSet = compressed.m_frameSets[frameSetIndex]
        samplePeriod = 1.0 / compressed.m_sampleFrequency
        baseTime = frameSet.m_baseFrame * samplePeriod

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_initialRBitpacked))

        for i in range(len(compressed.m_animRChannels)):
            jointIndex = compressed.m_animRChannels[i]
            key = AnimationKeyframe()
            key.m_keyTime = baseTime
            if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                key.m_keyData = frameSet.m_initialRRaw[i]
            elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3:
                key.m_keyData = DecompressQuat(frameSet.m_initialRSmallest3[i])
            elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                key.m_keyData = BitDecompressQuat(bitStream, compressed.m_packingSpecsR[i + 1])
            else:
                print("???? Unreachable")
                exit

            animation.m_jointAnimations[jointIndex].m_rotationAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_initialTBitpacked))

        for i in range(len(compressed.m_animTChannels)):
            jointIndex = compressed.m_animTChannels[i]
            key = AnimationKeyframe()
            key.m_keyTime = baseTime
            if compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                key.m_keyData = frameSet.m_initialTRaw[i]
            elif compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsT[i + 1])
            else:
                print("???? Unreachable")
                exit

            animation.m_jointAnimations[jointIndex].m_translationAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_initialSBitpacked))

        for i in range(len(compressed.m_animSChannels)):
            jointIndex = compressed.m_animSChannels[i]
            key = AnimationKeyframe()
            key.m_keyTime = baseTime
            if compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                key.m_keyData = frameSet.m_initialSRaw[i]
            elif compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsS[i + 1])
            else:
                print("???? Unreachable")
                exit

            animation.m_jointAnimations[jointIndex].m_scaleAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_initialUBitpacked))

        for i in range(len(compressed.m_animUChannels)):
            jointIndex = compressed.m_animUChannels[i]
            key = AnimationKeyframe()
            key.m_keyTime = baseTime
            if compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                key.m_keyData = frameSet.m_initialURaw[i]
            elif compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                key.m_keyData = BitDecompressFloat(bitStream, compressed.m_packingSpecsU[i + 1])
            else:
                print("???? Unreachable")
                exit

            animation.m_userChannelAnimations[channelIndex].m_animation.append(key)

        intraBits = NoeBitStream(reverseBitOrder(frameSet.m_intraBits))
        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_intraRBitpacked))

        intraRRawIndex = 0
        intraRSmallest3Index = 0
        for i in range(len(compressed.m_animRChannels)):
            for intraIndex in range(frameSet.m_numIntraFrames):
                if intraBits.readBits(1):
                    jointIndex = compressed.m_animRChannels[i]
                    key = AnimationKeyframe()
                    key.m_keyTime = (frameSet.m_baseFrame + 1 + intraIndex) * samplePeriod
                    if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                        key.m_keyData = frameSet.m_intraRRaw[intraRRawIndex]
                        intraRRawIndex += 1
                    elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3:
                        key.m_keyData = DecompressQuat(frameSet.m_intraRSmallest3[ intraRSmallest3Index])
                        intraRSmallest3Index += 1
                    elif compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                        key.m_keyData = BitDecompressQuat(bitStream, compressed.m_packingSpecsR[i + 1])
                    else:
                        print("???? Unreachable")
                        exit

                    animation.m_jointAnimations[jointIndex].m_rotationAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_intraTBitpacked))
        intraTRawIndex = 0
        for i in range(len(compressed.m_animTChannels)):
            for intraIndex in range(frameSet.m_numIntraFrames):
                if intraBits.readBits(1):
                    jointIndex = compressed.m_animTChannels[i]
                    key = AnimationKeyframe()
                    key.m_keyTime = (frameSet.m_baseFrame + 1 + intraIndex) * samplePeriod
                    if compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                        key.m_keyData = frameSet.m_intraTRaw[intraTRawIndex]
                        intraTRawIndex += 1
                    elif compressed.m_compressionInfo.m_compressionTypeTranslation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                        key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsT[i + 1])
                    else:
                        print("???? Unreachable")
                        exit

                    animation.m_jointAnimations[jointIndex].m_translationAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_intraSBitpacked))
        intraSRawIndex = 0
        for i in range(len(compressed.m_animSChannels)):
            for intraIndex in range(frameSet.m_numIntraFrames):
                if intraBits.readBits(1):
                    jointIndex = compressed.m_animSChannels[i]
                    key = AnimationKeyframe()
                    key.m_keyTime = (frameSet.m_baseFrame + 1 + intraIndex) * samplePeriod
                    if compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                        key.m_keyData = frameSet.m_intraSRaw[intraSRawIndex]
                        intraSRawIndex += 1
                    elif compressed.m_compressionInfo.m_compressionTypeScale == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                        key.m_keyData = BitDecompressVec3(bitStream, compressed.m_packingSpecsS[i + 1])
                    else:
                        print("???? Unreachable")
                        exit

                    animation.m_jointAnimations[jointIndex].m_scaleAnimation.append(key)

        if compressed.m_compressionInfo.m_compressionTypeRotation == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
            bitStream = NoeBitStream(reverseBitOrder(frameSet.m_intraUBitpacked))
        intraURawIndex = 0
        for i in range(len(compressed.m_animUChannels)):
            for intraIndex in range(frameSet.m_numIntraFrames):
                if intraBits.readBits(1):
                    jointIndex = compressed.m_animUChannels[i]
                    key = AnimationKeyframe()
                    key.m_keyTime = (frameSet.m_baseFrame + 1 + intraIndex) * samplePeriod
                    if compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_NONE:
                        key.m_keyData = frameSet.m_intraURaw[intraURawIndex]
                        intraURawIndex += 1
                    elif compressed.m_compressionInfo.m_compressionTypeUser == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
                        key.m_keyData = BitDecompressFloat(bitStream, compressed.m_packingSpecsU[i + 1])
                    else:
                        print("???? Unreachable")
                        exit

                    channel = animation.m_userChannelAnimations[channelIndex].m_animation.append(key)

    #detect stepped keys
    periodEpsilon = animation.m_period / 100.0
    for jointIndex in range(len(animation.m_jointAnimations)):
        joint = animation.m_jointAnimations[jointIndex]
        for frameIndex in range(1, len(joint.m_rotationAnimation)):
            prevFrame = joint.m_rotationAnimation[frameIndex - 1]
            currFrame = joint.m_rotationAnimation[frameIndex]
            if abs(currFrame.m_keyTime - prevFrame.m_keyTime) < periodEpsilon:
                prevFrame.m_keyFlags |= AnimationKeyframe.kKeyFrameStepped
        for frameIndex in range(1, len(joint.m_translationAnimation)):
            prevFrame = joint.m_translationAnimation[frameIndex - 1]
            currFrame = joint.m_translationAnimation[frameIndex]
            if abs(currFrame.m_keyTime - prevFrame.m_keyTime) < periodEpsilon:
                prevFrame.m_keyFlags |= AnimationKeyframe.kKeyFrameStepped
        for frameIndex in range(1, len(joint.m_scaleAnimation)):
            prevFrame = joint.m_scaleAnimation[frameIndex - 1]
            currFrame = joint.m_scaleAnimation[frameIndex]
            if abs(currFrame.m_keyTime - prevFrame.m_keyTime) < periodEpsilon:
                prevFrame.m_keyFlags |= AnimationKeyframe.kKeyFrameStepped

    for channelIndex in range(len(animation.m_userChannelAnimations)):
        channel = animation.m_userChannelAnimations[channelIndex]
        for frameIndex in range(1, len(channel.m_animation)):
            prevFrame = channel.m_animation[frameIndex - 1]
            currFrame = channel.m_animation[frameIndex]
            if abs(currFrame.m_keyTime - prevFrame.m_keyTime) < periodEpsilon:
                prevFrame.m_keyFlags |= AnimationKeyframe.kKeyFrameStepped

    return animation

def printAnimLog(pAnim, compressed, animation, skeleton, anim_path = None):
    print("EdgeAnimAnimation")
    if anim_path != None:
        print("File:                 %s" % anim_path)
    print("Endianness:           %s" % ("LittleEndian", "BigEndian")[pAnim.bigEndian])
    print("Size:                 %i bytes" % pAnim.length)
    print("Duration:             %f" % pAnim.duration)
    print("Buffer Size:          %d" % pAnim.evalBufferSizeRequired)
    print("Sample Frequency:     %f" % pAnim.sampleFrequency)
    print("Number of Joints:     %d" % pAnim.numJoints)
    print("Number of Channels:   %d" % pAnim.numUserChannels)
    print("Number of Frames:     %d" % pAnim.numFrames)
    print("Number of Frame Sets: %d" % pAnim.numFrameSets)

    print("Locomotion Delta:     %s" % ("Missing" , "Available")[compressed.m_enableLocoDelta])
    if compressed.m_enableLocoDelta:
        print("Quat:   %f, %f, %f, %f" % (compressed.m_locoDeltaQuat[0], compressed.m_locoDeltaQuat[1], compressed.m_locoDeltaQuat[2], compressed.m_locoDeltaQuat[3]))
        print("Trans:  %f, %f, %f" % (compressed.m_locoDeltaTrans[0], compressed.m_locoDeltaTrans[1], compressed.m_locoDeltaTrans[2]))

    print()

    print("Number of ConstantR:  %d (%s)" % (compressed.m_numConstRChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeRotation)))
    for i in range(compressed.m_numConstRChannels):
        print(compressed.m_constRChannels[i], end = " ")
    print()

    print("Number of ConstantT:  %d (%s)" % (compressed.m_numConstTChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeTranslation)))
    for i in range(compressed.m_numConstTChannels):
        print(compressed.m_constTChannels[i], end = " ")
    print()

    print("Number of ConstantS:  %d (%s)" % (compressed.m_numConstSChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeScale)))
    for i in range(compressed.m_numConstSChannels):
        print(compressed.m_constSChannels[i], end = " ")
    print()

    print("Number of ConstantU:  %d (%s)" % (compressed.m_numConstUChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeUser)))
    for i in range(compressed.m_numConstUChannels):
        print(compressed.m_constUChannels[i], end = " ")
    print()

    print("Number of AnimatedR:  %d (%s)" % (compressed.m_numAnimRChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeRotation)))
    for i in range(compressed.m_numAnimRChannels):
        print(compressed.m_animRChannels[i], end = " ")
    print()

    print("Number of AnimatedT:  %d (%s)" % (compressed.m_numAnimTChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeTranslation)))
    for i in range(compressed.m_numAnimTChannels):
        print(compressed.m_animTChannels[i], end = " ")
    print()

    print("Number of AnimatedS:  %d (%s)" % (compressed.m_numAnimSChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeScale)))
    for i in range(compressed.m_numAnimSChannels):
        print(compressed.m_animSChannels[i], end = " ")
    print()

    print("Number of AnimatedU:  %d (%s)" % (compressed.m_numAnimUChannels, strEdgeAnimCompressionType(compressed.m_compressionInfo.m_compressionTypeUser)))
    for i in range(compressed.m_numAnimUChannels):
        print(compressed.m_animUChannels[i], end = " ")
    print()

    print("Partial Weight Data:  %s" % ("Missing" , "Available")[compressed.m_enableWeights])
    if compressed.m_enableWeights:
        for jointIndex in range(compressed.m_numJoints):
            #print("Joint[%d]: 0x0x%d" % (jointIndex, compressed.m_jointWeights[jointIndex])) #Pretty sure something is broken
            print("Joint[%d - %s]: %s" % (jointIndex, skeleton[jointIndex].name, "0x{:X}".format(compressed.m_jointWeights[jointIndex])))
        for channelIndex in range(compressed.m_numUserChannels):
            #print("Channel[%d]: 0x0x%d" % (channelIndex, compressed.m_userChannelWeights[channelIndex])) #Same as above
            print("Channel[%d]: %s" % (channelIndex, "0x{:X}".format(compressed.m_userChannelWeights[channelIndex])))
    print()

    print("KeyFrames:")
    for jointIndex in range(len(animation.m_jointAnimations)):
        joint = animation.m_jointAnimations[jointIndex]
        numKeys = len(joint.m_rotationAnimation) + len(joint.m_translationAnimation) + len(joint.m_scaleAnimation)
        print("Joint[%d - %s]:" % (jointIndex, skeleton[jointIndex].name))
        if numKeys > 0:
            for key in joint.m_rotationAnimation:
                print("Quat:  %f, %f, %f, %f : %f %s" % (key.m_keyData[0], key.m_keyData[1], key.m_keyData[2], key.m_keyData[3], key.m_keyTime, strAnimationKeyframeFlags(key)))
                # if printEuler:
            for key in joint.m_translationAnimation:
                print("Trans:  %f, %f, %f: %f %s" % (key.m_keyData[0], key.m_keyData[1], key.m_keyData[2], key.m_keyTime, strAnimationKeyframeFlags(key)))
            for key in joint.m_scaleAnimation:
                print("Scale:  %f, %f, %f : %f %s" % (key.m_keyData[0], key.m_keyData[1], key.m_keyData[2], key.m_keyTime, strAnimationKeyframeFlags(key)))
        print()

    for channelIndex in range(len(animation.m_userChannelAnimations)):
        channel = animation.m_userChannelAnimations[channelIndex]
        print("Channel[%d]:" % channelIndex)
        if len(channel.m_animation) > 0:
            for i in range(len(channel.m_animation)):
                key = channel.m_animation[i]
                print("Value: %f : %f %s" % (key.m_keyData[0], key.m_keyTime, strAnimationKeyframeFlags(key)))
        print()




def BitDecompressFloat(bitStream, spec):
    packed = bitStream.readBits(spec.m_componentSpecs[0].GetNumBits())
    return spec.m_componentSpecs[0].Decode(packed)

def BitDecompressVec3(bitStream, spec):
    output = []
    for i in range(3):
        data = 0
        for bit in range(spec.m_componentSpecs[i].GetNumBits()):
            data <<= 1
            data |= bitStream.readBits(1)
        data = spec.m_componentSpecs[i].Decode(data)
        output.append(data)
    return output


def BitDecompressQuat(bitStream, spec):
    output = [0.0, 0.0, 0.0, 0.0]
    pack = []
    for i in range(3):
        data = 0
        for bit in range(spec.m_componentSpecs[i].GetNumBits()):
            data <<= 1
            data |= bitStream.readBits(1)
        pack.append(data)

    sumSq = 0
    c = 0
    for i in range(3):
        if i == spec.m_recomputeComponentIdx:
            c += 1
        output[c] = spec.m_componentSpecs[i].Decode(pack[i])
        sumSq += output[c] * output[c]
        c += 1

    if sumSq > 1:
        sumSq = 1

    output[spec.m_recomputeComponentIdx] = math.sqrt(1 - sumSq)

    return output

kSqrt2 = 1.414213562
kQuatScale = ((1 << 15) - 1) / kSqrt2
kQuatOffset = kQuatScale / kSqrt2

def DecompressQuat(input):
    output = [0.0, 0.0, 0.0, 0.0]
    a = (input >> 32) & ((1 << 15) - 1)
    b = (input >> 17) & ((1 << 15) - 1)
    c = (input >> 2) & ((1 << 15) - 1)
    idx = input & 3

    fa = (a - kQuatOffset) / kQuatScale
    fb = (b - kQuatOffset) / kQuatScale
    fc = (c - kQuatOffset) / kQuatScale
    fd = math.sqrt(1 - fa*fa - fb*fb - fc*fc)

    if idx == 0:
        output[0] = fd
        output[1] = fa
        output[2] = fb
        output[3] = fc
    elif idx == 1:
        output[0] = fa
        output[1] = fd
        output[2] = fb
        output[3] = fc
    elif idx == 2:
        output[0] = fa
        output[1] = fb
        output[2] = fd
        output[3] = fc
    elif idx == 3:
        output[0] = fa
        output[1] = fb
        output[2] = fc
        output[3] = fd

    return output

def EDGE_IS_ALIGNED(value, alignment):
    return (value & (alignment - 1)) == False

def EDGE_ALIGN(value, alignment, byteWidth):
    value *= byteWidth
    alignment *= byteWidth
    return (value + alignment - 1) & ~(alignment - 1)

def strEdgeAnimCompressionType(comp):
    if comp == EdgeAnimCompressionType.COMPRESSION_TYPE_BITPACKED:
        return "Bitpacked"
    elif comp == EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3:
        return "Smallest3"
    else:
        return "None"

def strAnimationKeyframeFlags(frame):
    if frame.m_keyFlags & AnimationKeyframe.kKeyFrameStepped:
        return "(Stepped)"
    else:
        return ""

def strEdgeAnimUserChannelFlags(flags):
    doClamp = bool(flags & EdgeAnimUserChannelFlags.EDGE_ANIM_USER_CHANNEL_FLAG_CLAMP01)
    doMinMax = bool(flags & EdgeAnimUserChannelFlags.EDGE_ANIM_USER_CHANNEL_FLAG_MINMAX)
    if doClamp and doMinMax:
        return "(clamp01,minmax)"
    elif doClamp:
        return "(clamp01)"
    elif doMinMax:
        return "(minmax)"
    else:
        return ""

def reverseBitOrder(input):
    if input == None:
        return None
    inputArray = list(input)
    output = b''
    for byte in inputArray:
        inverted_byte = int('{:08b}'.format(byte)[::-1],2).to_bytes(1, byteorder='big')
        output += inverted_byte
    return output
# main()

def unHash(hash):
    result = getNameFromHash(hash)
    if result == hash:
        return hex(hash)
    return result