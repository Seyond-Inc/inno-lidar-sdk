/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk_client/stage_client_deliver.h"
#include <utility>

#include "sdk_client/lidar_client.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"
#include "sdk_common/ring_id_converter_interface.h"  // Ensure the complete type is included
#include "utils/consumer_producer.h"

namespace innovusion {

StageClientDeliver::StageClientDeliver(InnoLidarClient *l) {
  lidar_ = l;

  stats_total_jobs_ = 0;
  stats_dropped_jobs_ = 0;
  stats_data_jobs_ = 0;
  stats_message_jobs_ = 0;
  stats_status_jobs_ = 0;
  stats_points_ = 0;
  stats_2nd_return_points_ = 0;
  stats_frames_ = 0;
  xyz_data_packet_ = reinterpret_cast<InnoDataPacket*>(xyz_data_packet_buf_);
  lidar_->add_config(&config_base_);
  config_.copy_from_src(&config_base_);
}

StageClientDeliver::~StageClientDeliver(void) {
  lidar_->remove_config(&config_base_);
  // print frame loss rate at the end
  summary_package_.print_inno_data_packet_loss_rate(0);
}

int StageClientDeliver::process(void *in_job, void *ctx, bool prefer) {
  StageClientDeliver *s = reinterpret_cast<StageClientDeliver *>(ctx);
  return s->process_job_(reinterpret_cast<InnoCommonHeader *>(in_job), prefer);
}

int StageClientDeliver::process_job_(InnoCommonHeader *pkt, bool prefer) {
  stats_total_jobs_++;

  // only update stats for InnoDataPacket
  if (pkt->version.magic_number == kInnoMagicNumberDataPacket &&
      (reinterpret_cast<InnoDataPacket *>(pkt)->type != INNO_ITEM_TYPE_MESSAGE &&
       reinterpret_cast<InnoDataPacket *>(pkt)->type != INNO_ITEM_TYPE_MESSAGE_LOG &&
       reinterpret_cast<InnoDataPacket *>(pkt)->type != INNO_FALCON_RING_ID_TABLE)) {
    summary_package_.inno_data_packet_receive_stats(reinterpret_cast<const InnoDataPacket &>(*pkt));
  }

  if (!prefer) {
    stats_dropped_jobs_++;
    if (stats_dropped_jobs_ % 10 == 1) {
      inno_log_warning("drop data in deliver stage.");
      print_stats();
    }
    if (stats_dropped_jobs_ % 100 == 1) {
      lidar_->cp_deliver_->print_stats();
    }
    lidar_->free_buffer_(pkt);
    return 0;
  }
  bool need_free_buffer = true;
  size_t n = pkt->size;
  inno_log_verify(n >= sizeof(InnoCommonHeader), "%" PRI_SIZELU " vs %" PRI_SIZELU, n, sizeof(InnoCommonHeader));
  if (pkt->version.magic_number == kInnoMagicNumberStatusPacket) {
    InnoStatusPacket *status_packet = reinterpret_cast<InnoStatusPacket *>(pkt);

    // xxx todo: handle different version
    if (n == sizeof(InnoStatusPacket)) {
      stats_status_jobs_++;
      if (lidar_->status_packet_callback_) {
        lidar_->status_packet_callback_(lidar_->handle_, lidar_->callback_context_, status_packet);
      }
      lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_STATUS, 1, n);
      // update ring_id table if mode changed
      if (status_packet->common.lidar_status == 0) {
        if (status_packet->common.lidar_mode != cur_lidar_mode_) {
          lidar_->update_ring_id_table(nullptr);
        }
        cur_lidar_mode_ = status_packet->common.lidar_mode;
      }
    } else {
      inno_log_warning("size mismatch %" PRI_SIZELU " %" PRI_SIZELU, n, sizeof(InnoStatusPacket));
    }
  } else if (pkt->version.magic_number == kInnoMagicNumberDataPacket) {
    InnoDataPacket *data_packet = reinterpret_cast<InnoDataPacket *>(pkt);
    if (n >= sizeof(InnoDataPacket)) {
      if (CHECK_SPHERE_POINTCLOUD_DATA(data_packet->type) || CHECK_XYZ_POINTCLOUD_DATA(data_packet->type)) {
        uint64_t start = InnoUtils::get_time_ns();
        uint64_t start_2 = 0;
        stats_data_jobs_++;
        uint64_t point_count_2nd_return = 0;

        // get anglehv_table
        if ((CHECK_CO_SPHERE_POINTCLOUD_DATA(data_packet->type)) && !lidar_->anglehv_init_) {
          char *table = new char[kInnoAngleHVTableMaxSize];
          lidar_->get_anglehv_table(reinterpret_cast<InnoDataPacket *>(table));
          delete[] table;
        }
        if (lidar_->data_packet_callback_) {
          // inno_log_debug("data callback %" PRI_SIZEU "", n);
          if ((lidar_->force_xyz_pointcloud_ || lidar_->callback_data_type_ == INNO_CALLBACK_XYZ_PACKET) &&
                                                   CHECK_SPHERE_POINTCLOUD_DATA(data_packet->type)) {
            InnoDataPacket *xyz_pkt = NULL;
            size_t buf_size = 0;
            if (!config_.enable_stage_client_deliver2) {
              xyz_pkt = xyz_data_packet_;
              buf_size = kMaxXyzDataPacketBufSize;
            } else {
              xyz_pkt = reinterpret_cast<InnoDataPacket *>(lidar_->alloc_buffer_(kMaxInnoDataBufferSize));
              buf_size = kMaxInnoDataBufferSize;
            }
            if (InnoDataPacketUtils::convert_to_xyz_pointcloud(
                    *data_packet, xyz_pkt, buf_size, config_.disable_do_crc,
                    reinterpret_cast<RingIdConverterInterface *>(lidar_->ring_id_converter_),
                    reinterpret_cast<char *>(
                        reinterpret_cast<InnoAngleHVTable *>(lidar_->anglehv_table_->payload)->table))) {
              start_2 = InnoUtils::get_time_ns();
              convert_xyz_mean_ms_.add((start_2 - start) / 1000000.0);
              if (!config_.enable_stage_client_deliver2) {
                lidar_->data_packet_callback_(lidar_->handle_, lidar_->callback_context_, xyz_pkt);
              } else {
                lidar_->add_deliver2_job_(reinterpret_cast<void *>(xyz_pkt));
              }
              point_count_2nd_return = InnoDataPacketUtils::get_points_count_2nd_return(*xyz_pkt);
            } else {
              inno_log_error("cannot convert data_packet");
            }
          } else if (lidar_->callback_data_type_ == INNO_CALLBACK_XYZ_FRAME &&
                     CHECK_SPHERE_POINTCLOUD_DATA(data_packet->type)) {
            InnoDataPacket *xyz_pkt = xyz_data_packet_;
            bool append = true;
            if (last_frame_idx_ != data_packet->idx) {
              // if the last packet is not the last sub frame, and the current packet is the next sub frame
              // it means the last packet is lost, or misordered of the packets, need deliver last frame
              if (xyz_pkt->is_last_sub_frame != 1 && (data_packet->idx - last_frame_idx_ == 1)) {
                inno_log_warning("drop last sub frame packet");
                lidar_->data_packet_callback_(lidar_->handle_, lidar_->callback_context_, xyz_pkt);
              }
              // if current packet is last frame packet,drop it
              if (last_frame_idx_ - data_packet->idx == 1) {
                lidar_->free_buffer_(pkt);
                return 0;
              }
              last_frame_idx_ = data_packet->idx;
              append = false;
            }
            InnoDataPacketUtils::convert_to_xyz_pointcloud(
                *data_packet, xyz_pkt, kMaxXyzDataPacketBufSize, config_.disable_do_crc,
                reinterpret_cast<RingIdConverterInterface *>(lidar_->ring_id_converter_),
                reinterpret_cast<char *>(reinterpret_cast<InnoAngleHVTable *>(lidar_->anglehv_table_->payload)->table),
                append);
            // last packet of the frame, deliver the frame
            if (data_packet->is_last_sub_frame) {
              xyz_pkt->is_last_sub_frame = 1;
              lidar_->data_packet_callback_(lidar_->handle_, lidar_->callback_context_, xyz_pkt);
              point_count_2nd_return = InnoDataPacketUtils::get_points_count_2nd_return(*xyz_pkt);
            }
            start_2 = InnoUtils::get_time_ns();
            convert_xyz_mean_ms_.add((start_2 - start) / 1000000.0);
          } else {
            start_2 = start;

            if (!config_.enable_stage_client_deliver2) {
              lidar_->data_packet_callback_(lidar_->handle_, lidar_->callback_context_, data_packet);
            } else {
              need_free_buffer = false;
              lidar_->add_deliver2_job_(reinterpret_cast<void *>(data_packet));
            }
          }
          callback_mean_ms_.add((InnoUtils::get_time_ns() - start_2) / 1000000.0);
        }
        lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_DATA, 1, n);
        int new_frame = 0;
        if (data_packet->is_last_sub_frame) {
          stats_frames_++;
          new_frame = 1;
          // print every 30 seconds
          if (stats_frames_ % (15 * 30) == 10) {
            print_stats();
            lidar_->cp_deliver_->print_stats();
          }
        }
        uint32_t point_count = InnoDataPacketUtils::get_points_count(*data_packet);
        lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_POINT, new_frame, point_count);
        stats_points_ += point_count;
        stats_2nd_return_points_ += point_count_2nd_return;
      } else if (data_packet->type == INNO_ITEM_TYPE_MESSAGE || data_packet->type == INNO_ITEM_TYPE_MESSAGE_LOG) {
        char *p_addr = reinterpret_cast<char *>(const_cast<InnoDataPacket *>(data_packet));
        InnoMessage *messages = reinterpret_cast<InnoMessage *>(p_addr + sizeof(InnoDataPacket));
        InnoMessage *m = &messages[0];
        stats_message_jobs_++;
        if (n != data_packet->item_size + sizeof(InnoDataPacket) || data_packet->item_size <= sizeof(InnoMessage) ||
            data_packet->item_number != 1 || data_packet->item_size != m->size) {
          inno_log_warning("invalid message n=%" PRI_SIZELU
                           " item_size=%u item_num=%u m_size=%u, "
                           "%" PRI_SIZELU " %" PRI_SIZELU,
                           n, data_packet->item_size, data_packet->item_number, m->size, sizeof(InnoDataPacket),
                           sizeof(InnoMessage));
        } else {
          // zero-terminate message
          bool do_external_callback = true;
          m->content[m->size - sizeof(InnoMessage) - 1] = 0;
          if (m->code == INNO_MESSAGE_CODE_NEW_START) {
            #if !(defined (_QNX_) || defined (_WIN32))
            if (!lidar_->faults_save_raw_.empty()) {
              lidar_->set_faults_save_raw(lidar_->faults_save_raw_);
              lidar_->set_attribute_string("udp_raw_port", std::to_string(lidar_->raw_receive_port_).c_str());
            }
            #endif
          } else if (m->code == INNO_MESSAGE_CODE_ROI_CHANGED) {
            // update ring-id table if using ring-id
            lidar_->update_ring_id_table(nullptr);
          } else if (m->code == INNO_MESSAGE_CODE_MAX_DISTANCE_CHECK_RESULT) {
            // maxdistance code, not callback to app
            do_external_callback = false;
          }
          if (lidar_->message_callback_external_ && do_external_callback) {
            // inno_log_debug("message callback");
            lidar_->message_callback_external_(lidar_->handle_, lidar_->callback_context_, 1,
                                               InnoMessageLevel(m->level), InnoMessageCode(m->code), m->content);
          }
          lidar_->stats_update_packet_bytes(ResourceStats::PACKET_TYPE_MESSAGE, 1, n);
        }
      } else if (data_packet->type == INNO_FALCON_RING_ID_TABLE) {
        lidar_->update_ring_id_table(data_packet);
      } else {
        inno_log_warning("unknow data_packet type %u", data_packet->type);
      }
    } else {
      inno_log_warning("message size mismatch %" PRI_SIZELU " %" PRI_SIZELU, n, sizeof(InnoDataPacket));
    }
  } else {
    inno_log_warning("bad message, size=%" PRI_SIZELU, n);
  }
  if (need_free_buffer) {
    lidar_->free_buffer_(pkt);
  }
  return 0;
}

void StageClientDeliver::print_stats() const {
  inno_log_info(
      "StageClientDeliever: "
      "convert_xyz mean/std/max/total=%.2fms/%.2f/%.2f/"
      "%" PRI_SIZEU
      " callback mean/std/max/total=%.2fms/%.2f/%.2f/%" PRI_SIZEU
      " total=%" PRI_SIZEU " total_dropped=%" PRI_SIZEU
      " data=%" PRI_SIZEU " message=%" PRI_SIZEU
      " status="
      "%" PRI_SIZEU " points=%" PRI_SIZEU " frames=%" PRI_SIZEU
      " points_2nd_return=%" PRI_SIZEU,
      convert_xyz_mean_ms_.mean(), convert_xyz_mean_ms_.std_dev(), convert_xyz_mean_ms_.max(),
      convert_xyz_mean_ms_.count(), callback_mean_ms_.mean(), callback_mean_ms_.std_dev(), callback_mean_ms_.max(),
      callback_mean_ms_.count(), stats_total_jobs_, stats_dropped_jobs_, stats_data_jobs_, stats_message_jobs_,
      stats_status_jobs_, stats_points_, stats_frames_, stats_2nd_return_points_);
}

const char *StageClientDeliver::get_name_(void) const {
  return lidar_->get_name();
}

StageClientDeliver2::StageClientDeliver2(InnoLidarClient *l) {
  lidar_ = l;
  stats_total_jobs_ = 0;
  stats_dropped_jobs_ = 0;
  stats_data_jobs_ = 0;
}

StageClientDeliver2::~StageClientDeliver2(void) {
}

int StageClientDeliver2::process(void *in_job, void *ctx, bool prefer) {
  StageClientDeliver2 *s = reinterpret_cast<StageClientDeliver2 *>(ctx);
  return s->process_job_(reinterpret_cast<InnoCommonHeader *>(in_job), prefer);
}

int StageClientDeliver2::process_job_(InnoCommonHeader *pkt, bool prefer) {
  stats_total_jobs_++;
  if (!prefer) {
    stats_dropped_jobs_++;
    if (stats_dropped_jobs_ % 10 == 1) {
      inno_log_warning("drop data in deliver2 stage.");
      print_stats();
    }
    if (stats_dropped_jobs_ % 100 == 1) {
      lidar_->cp_deliver2_->print_stats();
    }
    lidar_->free_buffer_(pkt);
    return 0;
  }
  size_t n = pkt->size;
  inno_log_verify(n >= sizeof(InnoCommonHeader), "%" PRI_SIZELU " vs %" PRI_SIZELU, n, sizeof(InnoCommonHeader));
  if (pkt->version.magic_number == kInnoMagicNumberDataPacket) {
    InnoDataPacket *data_packet = reinterpret_cast<InnoDataPacket *>(pkt);
    if (n >= sizeof(InnoDataPacket)) {
      if (CHECK_SPHERE_POINTCLOUD_DATA(data_packet->type) || CHECK_XYZ_POINTCLOUD_DATA(data_packet->type)) {
        if (lidar_->data_packet_callback_) {
          lidar_->data_packet_callback_(lidar_->handle_, lidar_->callback_context_, data_packet);
          lidar_->free_buffer_(pkt);
        } else {
          inno_log_panic("invalid! lidar_->data_packet_callback_ is NULL");
        }
      } else {
        inno_log_panic("invalid data_packet type %d", data_packet->type);
      }
    } else {
      inno_log_panic("invalid data_packet size %" PRI_SIZELU, n);
    }
  }
  return 0;
}

void StageClientDeliver2::print_stats() const {
  inno_log_info(
      "StageClientDeliever2: "
      "total=%" PRI_SIZEU " total_dropped=%" PRI_SIZEU "data=%" PRI_SIZEU,
      stats_total_jobs_, stats_dropped_jobs_, stats_data_jobs_);
}

}  // namespace innovusion
