import struct


# edgeanim_skeleton
class EdgeAnimUserChannelFlags:
    EDGE_ANIM_USER_CHANNEL_FLAG_CLAMP01 = 0x01
    EDGE_ANIM_USER_CHANNEL_FLAG_MINMAX = 0x02
    EDGE_ANIM_USER_CHANNEL_FLAG_CPT_SHIFT = 0x03
    EDGE_ANIM_USER_CHANNEL_FLAG_CPT_BIT1 = 0x08
    EDGE_ANIM_USER_CHANNEL_FLAG_CPT_BIT0 = 0x10


class EdgeAnimationSkeleton:
    def __init__(self, data):
        if data[0:4] == b'60SE':
            self.bigEndian = False
        elif data[0:4] == b'ES06':
            self.bigEndian = True
            print("Support for big endian file is very experimental")
        else:
            print("Skeleton File Incorrect")
            return

        self.length = len(data)
        self.data = data

        self.tag, \
        self.izeTotal, \
        self.sizeCustomData, \
        self.sizeNameHashes, \
        self.numJoints, \
        self.numUserChannels, \
        self.flags, \
        self.locomotionJointIndex, \
        self.offsetBasePose, \
        self.offsetJointLinkageMap, \
        self.offsetJointNameHashArray, \
        self.offsetUserChannelNameHashArray, \
        self.offsetUserChannelNodeNameHashArray, \
        self.offsetUserChannelFlagsArray, \
        self.offsetCustomData, \
        self.pad1, \
        self.pad2, \
        self.numJointLinkages, \
        self.jointLinkage0 = \
            struct.unpack(('<', '>')[self.bigEndian] + '4I4H10IH', data[:66])

        self.offsetBasePose += 0x18
        self.offsetJointLinkageMap += 0x1c
        self.offsetJointNameHashArray += 0x20
        self.offsetUserChannelNameHashArray += 0x24
        self.offsetUserChannelNodeNameHashArray += 0x28
        self.offsetUserChannelFlagsArray += 0x2c

class UserChannelInfo:
    def __init__(self, nodeNameHash = None, channelNameHash = None, componentIndex = None, flags = None):
        self.m_nodeNameHash = nodeNameHash
        self.m_channelNameHash = channelNameHash
        self.m_componentIndex = componentIndex
        self.m_flags = flags

class Skeleton:
    def __init__(self):
        self.m_locoJointIndex = None
        self.m_numJoints = None
        self.m_numUserChannels = None
        self.m_parentIndices = []
        self.m_basePose = []
        self.m_scaleCompensateFlags = []
        self.m_jointNameHashes = None
        self.m_userChannelInfoArray = []
        self.m_customData = None


# edgeanim_animation
class AnimationKeyframe:
    kKeyFrameStepped = (1 << 0)
    kKeyFrameMask = kKeyFrameStepped

    def __init__(self):
        self.m_keyTime = 0.0
        self.m_keyData = [0.0, 0.0, 0.0, 0.0]
        self.m_keyFlags = 0


class EdgeAnimAnimationEncodingFlags:
    EDGE_ANIM_FLAG_BIT_PACKED_R = 0x01
    EDGE_ANIM_FLAG_BIT_PACKED_T = 0x02
    EDGE_ANIM_FLAG_BIT_PACKED_S = 0x04
    EDGE_ANIM_FLAG_BIT_PACKED_U = 0x08
    EDGE_ANIM_FLAG_RAW_R = 0x10


class EdgeAnimAnimation:
    def __init__(self, data):
        if data[0:4] == b'80AE':
            self.bigEndian = False
        elif data[0:4] == b'EA08':
            self.bigEndian = True
            print("Support for big endian file is very experimental")
        else:
            print("Animation File Incorrect")
            return

        self.length = len(data)
        self.data = data

        self.tag, \
        self.duration, \
        self.sampleFrequency, \
        self.sizeHeader, \
        self.numJoints, \
        self.numFrames, \
        self.numFrameSets, \
        self.evalBufferSizeRequired, \
        self.numConstRChannels, \
        self.numConstTChannels, \
        self.numConstSChannels, \
        self.numConstUserChannels, \
        self.numAnimRChannels, \
        self.numAnimTChannels, \
        self.numAnimSChannels, \
        self.numAnimUserChannels, \
        self.numUserChannels, \
        self.sizeJointsWeightArray, \
        self.sizeUserChannelWeightArray, \
        self.flags, \
        self.offsetJointsWeightArray, \
        self.offsetUserChannelWeightArray, \
        self.offsetFrameSetDmaArray, \
        self.offsetFrameSetInfoArray, \
        self.offsetConstRData, \
        self.offsetConstTData, \
        self.offsetConstSData, \
        self.offsetConstUserData, \
        self.offsetPackingSpecs, \
        self.offsetCustomData, \
        self.sizeCustomData, \
        self.offsetLocomotionDelta, \
        self.channelTables0 = \
            struct.unpack(('<', '>')[self.bigEndian] + 'I2f16H13IH', data[:98])

        self.offsetFrameSetDmaArray += 0x38
        self.offsetFrameSetInfoArray += 0x3C
        self.offsetConstRData += 0x40
        self.offsetConstTData += 0x44
        self.offsetConstSData += 0x48
        self.offsetConstUserData += 0x4C
        self.offsetPackingSpecs += 0x50


class EdgeAnimFrameSetInfo:
    def __init__(self, data, endianSwap):
        self.baseFrame , self.numIntraFrames = struct.unpack(('<', '>')[endianSwap] + 'HH', data[:4])


#libedgeanimtool_animation
class Animation:
    def __init__(self):
        self.m_startTime = 0.0
        self.m_endTime = 0.0
        self.m_period = 0.0
        self.m_numFrames = 0
    
        self.m_enableLocoDelta = False
        self.m_locoDeltaQuat = None
        self.m_locoDeltaTrans = None
    
        self.m_jointAnimations = []
        self.m_userChannelAnimations = []

        self.m_enableWeights = None
        self.m_jointWeights = None
        self.m_userChannelWeights = None



class JointAnimation:
    def __init__(self):
        self.m_jointName = None
        self.m_jointWeight = None
    
        self.m_rotationAnimation = []
        self.m_translationAnimation = []
        self.m_scaleAnimation = []


class JointFrameSet:
    def __init__(self):
        self.m_initialRData = None
        self.m_initialTData = None
        self.m_initialSData = None
    
        self.m_intraRDataFrames = None
        self.m_intraTDataFrames = None
        self.m_intraSDataFrames = None
    
        self.m_hasRFrame = None
        self.m_hasTFrame = None
        self.m_hasSFrame = None


class UserChannelAnimation:
    def __init__(self):
        self.m_nodeName = None
        self.m_channelName = None
        self.m_weight = None
    
        self.m_animation = []


class UserChannelFrameSet:
    def __init__(self):
        self.m_initialData = None
        self.m_intraDataFrames = None
        self.m_hasFrame = None


class FrameSet:
    def __init__(self):
        self.m_baseFrame = None
        self.m_numIntraFrames = None


class EdgeAnimCompressionType:
    COMPRESSION_TYPE_NONE = 0
    COMPRESSION_TYPE_SMALLEST_3 = 1
    COMPRESSION_TYPE_BITPACKED = 2


class CompressionInfo:
    def __init__(self):
        self.m_maxEvalBufferSize = None
    
        self.m_compressionTypeRotation = EdgeAnimCompressionType.COMPRESSION_TYPE_SMALLEST_3
        self.m_compressionTypeTranslation = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE
        self.m_compressionTypeScale = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE
        self.m_compressionTypeUser = EdgeAnimCompressionType.COMPRESSION_TYPE_NONE
    
        self.m_defaultToleranceRotation = 0.001
        self.m_defaultToleranceTranslation = 0.001
        self.m_defaultToleranceScale = 0.001
        self.m_defaultToleranceUser = 0.001
    
        self.m_jointTolerancesRotation = None
        self.m_jointTolerancesTranslation = None
        self.m_jointTolerancesScale = None
        self.m_userChannelTolerances = None


class CompressedFrameSet:
    def __init__(self):
        self.m_baseFrame = None
        self.m_numIntraFrames = None
    
        self.m_initialRRaw = []
        self.m_initialRSmallest3 = []
        self.m_initialTRaw = []
        self.m_initialSRaw = []
        self.m_initialURaw = []
    
        self.m_initialRBitpacked = None
        self.m_initialTBitpacked = None
        self.m_initialSBitpacked = None
        self.m_initialUBitpacked = None
    
        self.m_intraBits = b''
    
        self.m_intraRRaw = []
        self.m_intraRSmallest3 = []
        self.m_intraTRaw = []
        self.m_intraSRaw = []
        self.m_intraURaw = []
    
        self.m_intraRBitpacked = None
        self.m_intraTBitpacked = None
        self.m_intraSBitpacked = None
        self.m_intraUBitpacked = None


class CompressedAnimation:
    def __init__(self):
        self.m_duration = None
        self.m_sampleFrequency = None
        self.m_numJoints = None
        self.m_numUserChannels = None
        self.m_numFrames = None
        self.m_numFrameSets = None
        self.m_evalBufferSizeRequired = None
    
        self.m_enableLocoDelta = None
        self.m_locoDeltaQuat = None
        self.m_locoDeltaTrans = None
    
        self.m_customData = None
    
        self.m_enableWeights = None
        self.m_jointWeights = None
        self.m_userChannelWeights = None
    
        self.m_numConstRChannels = None
        self.m_numConstTChannels = None
        self.m_numConstSChannels = None
        self.m_numConstUChannels = None
        self.m_numAnimRChannels = None
        self.m_numAnimTChannels = None
        self.m_numAnimSChannels = None
        self.m_numAnimUChannels = None
    
        self.m_compressionInfo = None
    
        self.m_packingSpecsR = []
        self.m_packingSpecsT = []
        self.m_packingSpecsS = []
        self.m_packingSpecsU = []
    
        self.m_constRChannels = None
        self.m_constTChannels = None
        self.m_constSChannels = None
        self.m_constUChannels = None
        self.m_animRChannels = None
        self.m_animTChannels = None
        self.m_animSChannels = None
        self.m_animUChannels = None
    
        self.m_constRRaw = []
        self.m_constRSmallest3 = []
        self.m_constTRaw = []
        self.m_constSRaw = []
        self.m_constURaw = []
    
        self.m_constRBitpacked = None
        self.m_constTBitpacked = None
        self.m_constSBitpacked = None
        self.m_constUBitpacked = None
    
        self.m_frameSets = []


#libedgeanimtool_common
class Joint:
    def __init__(self, data, bigEndian = False):
        self.m_rotation = struct.unpack(('<', '>')[bigEndian] + '4f', data[:16])
        self.m_translation = struct.unpack(('<', '>')[bigEndian] + '4f', data[16:32])
        self.m_scale = struct.unpack(('<', '>')[bigEndian] + '4f', data[32:48])

def Reverse(input, endianSwap): #Fuck! How can I tell it's short???? Gonna ignore this for now lol, no one uses Big Endian
    return input
    # if endianSwap:
    #     if isinstance(input, list):
    #         for element in input:
    #             element = Reverse(element, true)
    #     else:


#libedgeanimtool_bitpacking
class KeyframePackingSpec:
    def __init__(self, packedSpec):
        self.m_componentSpecs = [None, None, None]
        self.m_componentSpecs[0] = ComponentPackingSpec((packedSpec >> 31) & 1, (packedSpec >> 27) & 15, (packedSpec >> 22) & 31)
        self.m_componentSpecs[1] = ComponentPackingSpec((packedSpec >> 21) & 1, (packedSpec >> 17) & 15, (packedSpec >> 12) & 31)
        self.m_componentSpecs[2] = ComponentPackingSpec((packedSpec >> 11) & 1, (packedSpec >> 7) & 15, (packedSpec >> 2) & 31)
        self.m_recomputeComponentIdx = packedSpec & 3

    def GetNumBits(self):
        return self.m_componentSpecs[0].GetNumBits() + self.m_componentSpecs[1].GetNumBits() + self.m_componentSpecs[2].GetNumBits()


class ComponentPackingSpec:
    def __init__(self, s, e, m):
        self.m_numSignBits = s
        self.m_numExponentBits = e
        self.m_numMantissaBits = m

    def GetNumBits(self):
        return self.m_numSignBits + self.m_numExponentBits + self.m_numMantissaBits

    def Decode(self, bits):
        res = 0.0
        maskTable = [
            0x00000000,
            0x00000001, 0x00000003, 0x00000007, 0x0000000F,
            0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
            0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF,
            0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
            0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF,
            0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
            0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF,
            0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF]
        signMaskTable = [
            0x00000001, 0x00000002, 0x00000004, 0x00000008,
            0x00000010, 0x00000020, 0x00000040, 0x00000080,
            0x00000100, 0x00000200, 0x00000400, 0x00000800,
            0x00001000, 0x00002000, 0x00004000, 0x00008000,
            0x00010000, 0x00020000, 0x00040000, 0x00080000,
            0x00100000, 0x00200000, 0x00400000, 0x00800000,
            0x01000000, 0x02000000, 0x04000000, 0x08000000,
            0x10000000, 0x20000000, 0x40000000, 0x80000000]
        signExtendTable = [
            0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFC, 0xFFFFFFF8,
            0xFFFFFFF0, 0xFFFFFFE0, 0xFFFFFFC0, 0xFFFFFF80,
            0xFFFFFF00, 0xFFFFFE00, 0xFFFFFC00, 0xFFFFF800,
            0xFFFFF000, 0xFFFFE000, 0xFFFFC000, 0xFFFF8000,
            0xFFFF0000, 0xFFFE0000, 0xFFFC0000, 0xFFF80000,
            0xFFF00000, 0xFFE00000, 0xFFC00000, 0xFF800000,
            0xFF000000, 0xFE000000, 0xFC000000, 0xF8000000,
            0xF0000000, 0xE0000000, 0xC0000000, 0x80000000,
            0x00000000]
        expBiasTable = [128, 127, 126, 124, 120, 112, 96,  64,  0, -128, -384]

        # Floating point mode
        if self.m_numExponentBits:
            m = bits & maskTable[self.m_numMantissaBits]
            e = struct.unpack('i', ((bits >> self.m_numMantissaBits) & maskTable[self.m_numExponentBits]).to_bytes(4, byteorder='little'))[0] #Not sure y it's int32 not uint32, just cast it in case
            s = (bits >> (self.m_numMantissaBits + self.m_numExponentBits)) & maskTable[self.m_numSignBits]

            if self.m_numExponentBits:
                e += expBiasTable[self.m_numExponentBits]

            if (self.m_numMantissaBits <= 23):
                m <<= (23 - self.m_numMantissaBits)
            else:
                m >>= self.m_numMantissaBits - 23

            # Clamp exponent - is it really necessary?
            if e > 0x7FFFFFF:
                e = 0
                m = 0
                s = 0
            elif e >= ( 1 << 8 ):
                e = ( 1 << 8 ) - 1
                m = 0xffffffff >> ( 32 - 23 )

            self.m_floatBits  = 0
            self.m_floatBits |= s << 31
            self.m_floatBits |= e << 23
            self.m_floatBits |= m

            res = struct.unpack('f', self.m_floatBits.to_bytes(4, 'little'))[0]

        # Fixed point
        elif self.m_numMantissaBits:
            flim = maskTable[self.m_numMantissaBits]

            # Signed fixed point
            if self.m_numSignBits:
                # sign extend?
                val = bits
                if val & signMaskTable [self.m_numMantissaBits]:
                    val |= signExtendTable[self.m_numMantissaBits]
                    val = struct.unpack('i', val.to_bytes(4, byteorder='little'))[0]
                res = val / flim
            else: # Unsigned fixed point
                res =  bits / flim
        # print("Decode - %d -> %f (%d, %d, %d)" % (bits, res, self.m_numSignBits, self.m_numExponentBits, self.m_numMantissaBits))
        return res


#edge_dma
class EdgeDmaListElement:
    def __init__(self, data, endianSwap):
        self.size, self.eal = struct.unpack(('<', '>')[endianSwap] + 'II', data[:8])


