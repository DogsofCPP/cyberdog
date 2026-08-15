// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef LCMTESTCS__LCMTESTCS_HPP_
#define LCMTESTCS__LCMTESTCS_HPP_

#include <lcm/lcm_coretypes.h>

#include <string>

namespace lcmtestcs
{

class lcmtestcs
{
public:
  std::string cmd;

  std::string data;

public:
  /**
   * Encode a message into binary form.
   *
   * @param buf The output buffer.
   * @param offset Encoding starts at thie byte offset into @p buf.
   * @param maxlen Maximum number of bytes to write.  This should generally be
   *  equal to getEncodedSize().
   * @return The number of bytes encoded, or <0 on error.
   */
  inline int encode(void * buf, int offset, int maxlen) const;

  /**
   * Check how many bytes are required to encode this message.
   */
  inline int getEncodedSize() const;

  /**
   * Decode a message from binary form into this instance.
   *
   * @param buf The buffer containing the encoded message.
   * @param offset The byte offset into @p buf where the encoded message starts.
   * @param maxlen The maximum number of bytes to read while decoding.
   * @return The number of bytes decoded, or <0 if an error occured.
   */
  inline int decode(const void * buf, int offset, int maxlen);

  /**
   * Retrieve the 64-bit fingerprint identifying the structure of the message.
   * Note that the fingerprint is the same for all instances of the same
   * message type, and is a fingerprint on the message type definition, not on
   * the message contents.
   */
  inline static int64_t getHash();

  /**
   * Returns "lcmtestcs"
   */
  inline static const char * getTypeName();

  // LCM support functions. Users should not call these
  inline int _encodeNoHash(void * buf, int offset, int maxlen) const;
  inline int _getEncodedSizeNoHash() const;
  inline int _decodeNoHash(const void * buf, int offset, int maxlen);
  inline static uint64_t _computeHash(const __lcm_hash_ptr * p);
};

int lcmtestcs::encode(void * buf, int offset, int maxlen) const
{
  int pos = 0, tlen;
  int64_t hash = getHash();

  tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
  if (tlen < 0) {return tlen;} else {pos += tlen;}

  tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0) {return tlen;} else {pos += tlen;}

  return pos;
}

int lcmtestcs::decode(const void * buf, int offset, int maxlen)
{
  int pos = 0, thislen;

  int64_t msg_hash;
  thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
  if (thislen < 0) {return thislen;} else {pos += thislen;}
  if (msg_hash != getHash()) {return -1;}

  thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
  if (thislen < 0) {return thislen;} else {pos += thislen;}

  return pos;
}

int lcmtestcs::getEncodedSize() const
{
  return 8 + _getEncodedSizeNoHash();
}

int64_t lcmtestcs::getHash()
{
  static int64_t hash = static_cast<int64_t>(_computeHash(NULL));
  return hash;
}

const char * lcmtestcs::getTypeName()
{
  return "lcmtestcs";
}

int lcmtestcs::_encodeNoHash(void * buf, int offset, int maxlen) const
{
  int pos = 0, tlen;

  char * cmd_cstr = const_cast<char *>(this->cmd.c_str());
  tlen = __string_encode_array(
    buf, offset + pos, maxlen - pos, &cmd_cstr, 1);
  if (tlen < 0) {return tlen;} else {pos += tlen;}

  char * data_cstr = const_cast<char *>(this->data.c_str());
  tlen = __string_encode_array(
    buf, offset + pos, maxlen - pos, &data_cstr, 1);
  if (tlen < 0) {return tlen;} else {pos += tlen;}

  return pos;
}

int lcmtestcs::_decodeNoHash(const void * buf, int offset, int maxlen)
{
  int pos = 0, tlen;

  int32_t __cmd_len__;
  tlen = __int32_t_decode_array(
    buf, offset + pos, maxlen - pos, &__cmd_len__, 1);
  if (tlen < 0) {return tlen;} else {pos += tlen;}
  if (__cmd_len__ > maxlen - pos) {return -1;}
  this->cmd.assign(
    static_cast<const char *>(buf) + offset + pos, __cmd_len__ - 1);
  pos += __cmd_len__;

  int32_t __data_len__;
  tlen = __int32_t_decode_array(
    buf, offset + pos, maxlen - pos, &__data_len__, 1);
  if (tlen < 0) {return tlen;} else {pos += tlen;}
  if (__data_len__ > maxlen - pos) {return -1;}
  this->data.assign(
    static_cast<const char *>(buf) + offset + pos, __data_len__ - 1);
  pos += __data_len__;

  return pos;
}

int lcmtestcs::_getEncodedSizeNoHash() const
{
  int enc_size = 0;
  enc_size += this->cmd.size() + 4 + 1;
  enc_size += this->data.size() + 4 + 1;
  return enc_size;
}

uint64_t lcmtestcs::_computeHash(const __lcm_hash_ptr *)
{
  uint64_t hash = 0x70f5fbbfcf2fd982LL;
  return (hash << 1) + ((hash >> 63) & 1);
}

}  // namespace lcmtestcs

#endif  // LCMTESTCS__LCMTESTCS_HPP_
