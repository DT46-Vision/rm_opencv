import struct
import zlib

# 打包数据
packet = struct.pack(
    "<Bfff",
    0x5A,
    12.4,
    56.1,
    100.0,
)

# 计算校验和
checksum = zlib.crc32(packet) & 0xFFFF  # 取低16位作为校验和

# 将校验和添加到数据包中
packet_with_checksum = packet + struct.pack("<H", checksum)

# 打印最终数据包
print(packet_with_checksum)

# 解包数据
unpacked_data = struct.unpack("<BfffH", packet_with_checksum)
print(unpacked_data)

# 打印校验和
print("Checksum:", checksum)