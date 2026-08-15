"""LCM type definitions generated for file_send_lcmt.

struct file_send_lcmt {
    string data;
}
"""
try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct


class file_send_lcmt(object):
    __slots__ = ["data"]

    __typenames__ = ["string"]

    __dimensions__ = [None]

    def __init__(self):
        self.data = ""

    def encode(self):
        buf = BytesIO()
        buf.write(file_send_lcmt._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        data = self.data
        if isinstance(data, bytes):
            data_encoded = data
        else:
            data_encoded = data.encode('utf-8')
        buf.write(struct.pack('>I', len(data_encoded) + 1))
        buf.write(data_encoded)
        buf.write(b"\0")

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != file_send_lcmt._get_packed_fingerprint():
            raise ValueError("Decode error")
        return file_send_lcmt._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = file_send_lcmt()
        data_len = struct.unpack('>I', buf.read(4))[0]
        self.data = buf.read(data_len)[:-1].decode('utf-8', 'replace')
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if file_send_lcmt in parents:
            return 0
        tmphash = (0x90df9b84cdceaf0a) & 0xffffffffffffffff
        tmphash = (((tmphash << 1) & 0xffffffffffffffff) +
                   (tmphash >> 63)) & 0xffffffffffffffff
        return tmphash

    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if file_send_lcmt._packed_fingerprint is None:
            file_send_lcmt._packed_fingerprint = struct.pack(
                ">Q", file_send_lcmt._get_hash_recursive([]))
        return file_send_lcmt._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct."""
        return struct.unpack(">Q", file_send_lcmt._get_packed_fingerprint())[0]
