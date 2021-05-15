#ifndef _PROTOCOL_C
#define _PROTOCOL_C

#include "protocol.h"
#include "crc.hpp"
#include "string.h"
#include <sstream>

struct null_streambuf : public std::streambuf
{
  void overflow(char c) {}
};

static null_streambuf null_buf;
static std::ostream cnull(&null_buf);
#define LOG_WARNING cnull

ProtocolProvider protocol_provider;

ProtocolConsumer::ProtocolConsumer() : default_handler(nullptr), index_(0), unpack_step(STEP_HEADER_SOF)
{
}

void ProtocolConsumer::unpack_process(uint8_t *data, uint16_t length)
{
  read_buff_index_ = 0;
  if (length > 0)
  {
    while (length--)
    {
      byte_ = data[read_buff_index_++];
      switch (unpack_step)
      {
      case STEP_HEADER_SOF:
      {
        if (byte_ == DN_REG_ID || byte_ == UP_REG_ID)
        {
          protocol_packet_[index_++] = byte_;
          unpack_step = STEP_LENGTH_LOW;
        }
        else
        {
          index_ = 0;
        }
      }
      break;
      case STEP_LENGTH_LOW:
      {
        data_length_ = byte_;
        protocol_packet_[index_++] = byte_;
        unpack_step = STEP_LENGTH_HIGH;
      }
      break;
      case STEP_LENGTH_HIGH:
      {
        data_length_ |= (byte_ << 8);
        protocol_packet_[index_++] = byte_;
        if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN))
        {
          unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          LOG_WARNING << "Data length too big";
          unpack_step = STEP_HEADER_SOF;
          index_ = 0;
        }
      }
      break;
      case STEP_FRAME_SEQ:
      {
        protocol_packet_[index_++] = byte_;
        unpack_step = STEP_HEADER_CRC8;
      }
      break;
      case STEP_HEADER_CRC8:
      {
        protocol_packet_[index_++] = byte_;
        bool crc8_result = verify_crc8_checksum(protocol_packet_, HEADER_LEN);
        if (!crc8_result)
        {
          LOG_WARNING << "CRC 8 error";
        }
        if ((index_ == HEADER_LEN) && crc8_result)
        {
          if (index_ < HEADER_LEN)
          {
            LOG_WARNING << "CRC 8 index less.";
          }
          unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          unpack_step = STEP_HEADER_SOF;
          index_ = 0;
        }
      }
      break;
      case STEP_DATA_CRC16:
      {
        if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN))
        {
          protocol_packet_[index_++] = byte_;
        }
        else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN))
        {
          LOG_WARNING << "Index Beyond";
        }
        if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN))
        {
          unpack_step = STEP_HEADER_SOF;
          index_ = 0;
          if (verify_crc16_checksum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN))
          {
            handle_frame((frame_header_t *)protocol_packet_);
          }
          else
          {
            LOG_WARNING << "CRC16 error";
          }
        }
      }
      break;
      default:
      {
        LOG_WARNING << "Unpack not well";
        unpack_step = STEP_HEADER_SOF;
        index_ = 0;
      }
      break;
      }
    }
  }
}

ProtocolConsumer::Handler ProtocolConsumer::register_handler(uint16_t cmd_id, Handler handler)
{
  auto iter = handlers.find(cmd_id);
  if (iter == handlers.end())
  {
    handlers[cmd_id] = handler;
    return nullptr;
  }
  else
  {
    Handler ret = iter->second;
    handlers[cmd_id] = handler;
    return ret;
  }
}

ProtocolConsumer::Handler ProtocolConsumer::unregister_handler(uint16_t cmd_id)
{
  auto iter = handlers.find(cmd_id);
  if (iter == handlers.end())
  {
    return nullptr;
  }
  else
  {
    Handler ret = iter->second;
    handlers.erase(iter);
    return ret;
  }
}

void ProtocolConsumer::set_default_handler(DefaultHandler handler)
{
  default_handler = handler;
}

void ProtocolConsumer::handle_frame(frame_header_t *packet)
{
  uint16_t cmd_id = *(uint16_t *)((uint8_t *)packet + HEADER_LEN);
  uint8_t *data = (uint8_t *)packet + HEADER_LEN + CMD_LEN;

  auto iter = handlers.find(cmd_id);
  if (iter != handlers.end())
    iter->second(data, packet->data_length);
  else if (default_handler)
    default_handler(cmd_id, data, packet->data_length);
}

uint16_t ProtocolProvider::pack(uint8_t *buffer, uint8_t sof, uint16_t cmd_id, uint8_t *data, uint16_t length)
{
  uint16_t frame_len = HEADER_LEN + CMD_LEN + length + CRC_LEN;

  frame_header_t *header = (frame_header_t *)buffer;
  header->sof = sof;
  header->data_length = length;
  header->seq = 0;
  append_crc8_checksum(buffer, HEADER_LEN);

  memcpy(buffer + HEADER_LEN, (uint8_t *)&cmd_id, CMD_LEN);
  memcpy(buffer + HEADER_LEN + CMD_LEN, data, length);
  append_crc16_checksum(buffer, frame_len);
  return frame_len;
}

#endif