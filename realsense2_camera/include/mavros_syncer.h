#ifndef REALSENSE2_CAMERA_MAVROS_SYNCER_H
#define REALSENSE2_CAMERA_MAVROS_SYNCER_H

#include "ros/ros.h"
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <geometry_msgs/PointStamped.h>
#include <mutex>
#include <tuple>

// Note on multi threading:
//      To avoid any confusion and non-defined state, this class locks a mutex for every function call
//      that is not const. This is due to the fact that many callbacks can happen simultaneously, especially
//      on callback based drivers such as the realsense. If not locked properly, this can lead to weird states.

namespace mavros_syncer {

template<typename t_channel_id, typename t_cache>
class MavrosSyncer {
    // callback definition for processing buffered frames
    typedef boost::function<void(const t_channel_id &channel,
                                 const ros::Time &new_stamp,
                                 const std::shared_ptr<t_cache> &cal)> caching_callback;

    // internal representation of a buffered frame
    // t_cache is the external representation
    typedef struct {
        uint32_t seq;
        ros::Time old_stamp;
        ros::Time arrival_stamp;
        std::shared_ptr<t_cache> frame;
        double exposure;
    } frame_buffer_type;

 public:

    enum SyncMode {
        None = 0,
        Trigger,
        Capture,
    };

    // TODO
    enum SyncState {
        Uninitialised = 0,
        WaitForSync,        // Waiting to receive trigger/capture events
        Synchronised,       // Currently in synchronisation
    };

    MavrosSyncer(const std::set<t_channel_id> &channel_set) :
            channel_set_(channel_set),
            state_(SyncState::Uninitialised) {
        ROS_DEBUG_STREAM(log_prefix_ << " Initialized with " << channel_set_.size() << " channels.");
        for (t_channel_id id : channel_set_) {
            trigger_buffer_[id].clear();
        }
    }

    void setup(const caching_callback &callback, int fps, double time_offset, int sync_mode) {

        sync_mode_ = sync_mode;
        sequence_offset_ = 0;
        time_offset_ = time_offset;
        state_ = SyncState::Uninitialised;
        restamp_callback_ = callback;

        camera_event_sub_ = nh_.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 100,
                                     &MavrosSyncer::cameraEventCallback, this);

        const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        const std::string mavros_trig_interval_srv = "/mavros/cmd/trigger_interval";

        if (mode_ == SyncMode::Trigger) {
            // Set up external camera triggering
            if (ros::service::exists(mavros_trig_control_srv, false) && 
                ros::service::exists(mavros_trig_interval_srv, false)) {

                // Disable trigger initially
                mavros_msgs::CommandTriggerControl req_control;
                req_control.request.trigger_enable = false;
                req_control.request.sequence_reset = true;
                req_control.request.trigger_pause = false;
                ros::service::call(mavros_trig_control_srv, req_control);

                // Set trigger cycle time
                mavros_msgs::CommandTriggerInterval req_interval;
                req_interval.request.cycle_time = 1000.0/fps;
                req_interval.request.integration_time = -1.0;
                ros::service::call(mavros_trig_interval_srv, req_interval);

                ROS_INFO("Set mavros trigger interval to %f! Success? %d Result? %d",
                                 1000.0/fps, req_interval.response.success, req_interval.response.result);
            } else {
                ROS_ERROR("Camera trigger setup services not available!");
            }
        }
    }

    void start() {
        std::lock_guard<std::mutex> lg(mutex_);

        if (state_ != SyncState::Uninitialized) {
            // Already started, ignore
            return;
        }

        for (t_channel_id id : channel_set_) {
            // Clear buffers in case we are re-initialising
            trigger_buffer_[id].clear();
            frame_buffer_[id].frame.reset(); // TODO better way? clear whole buffer?
        }

        // Reset sequence number and enable triggering
        sequence_offset_ = 0;
        if (inter_cam_sync_mode_ == 2) { 
            const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
            mavros_msgs::CommandTriggerControl req_enable;
            req_enable.request.trigger_enable = true;
            req_enable.request.sequence_reset = true;
            req_enable.request.trigger_pause = false;
            ros::service::call(mavros_trig_control_srv, req_enable);

            ROS_INFO_STREAM(log_prefix_ << " Started triggering.");

            ros::Duration(1.0).sleep(); // wait for the realsense to align its exposure to the external triggering
        }

        state_ = WaitForSync;
    }

    bool channelValid(const t_channel_id &channel) const {
        return channel_set_.count(channel) == 1;
    }

    void cacheFrame(const t_channel_id &channel, const uint32_t seq, const ros::Time &original_stamp, double exposure,
                                    const std::shared_ptr<t_cache> frame) {

        if (!channelValid(channel)) {
            ROS_WARN_STREAM_ONCE(log_prefix_ << "cacheFrame called for invalid channel.");
            return;
        }

        if(frame_buffer_[channel].frame){
            // TODO : this runs the publisher?
            ros::spinOnce();
        }

        if (frame_buffer_[channel].frame) {
            ROS_WARN_STREAM_THROTTLE(1, log_prefix_ << 
                "Overwriting image buffer! Make sure you're getting Timestamps from mavros.");
            // commented so that frames are only published if they were matched to a valid stamp
            // restamp_callback_(channel, frame_buffer_[channel].old_stamp, frame_buffer_[channel].frame);
        }

        // Buffer the frame
        frame_buffer_[channel].frame = frame;
        frame_buffer_[channel].old_stamp = original_stamp; //store stamp that was reconstructed by ros-realsense
        frame_buffer_[channel].arrival_stamp = ros::Time::now();
        frame_buffer_[channel].seq = seq; // TODO : where this from?
        frame_buffer_[channel].exposure = exposure;
        ROS_DEBUG_STREAM(log_prefix_ << "Buffered frame, seq: " << seq);
    }

    bool syncOffset(const t_channel_id &channel, const uint32_t seq, const ros::Time &old_stamp) {
        // no lock_guard as this function is only called within locked scopes

        // Get the first from the sequence time map.
        auto it = trigger_buffer_[channel].rbegin();
        int32_t mavros_sequence = it->first;

        // Get offset between first frame sequence and mavros
        sequence_offset_ = mavros_sequence - static_cast<int32_t>(seq);

        double delay = old_stamp.toSec() - it->second.toSec();


        ROS_INFO(
                "%sNew header offset determined for channel %i: %d, from %d to %d, timestamp "
                "correction: %f seconds.",
                log_prefix_.c_str(), channel.first,
                sequence_offset_, it->first, seq,
                delay);

        frame_buffer_[channel].frame.reset();
        trigger_buffer_[channel].clear();

        state_ = synced;
        return true;

    }

    // Match an incoming frame to a buffered hardware event (trigger/capture) TODO misleading name?
    bool lookupEvent(const t_channel_id &channel, const uint32_t frame_seq,
                            const ros::Time &old_stamp, double exposure, 
                            ros::Time *new_stamp) {
        std::lock_guard<std::mutex> lg(mutex_);

        if (!channelValid(channel)) {
            return false;
        }

        ROS_INFO_STREAM(log_prefix_ << "Received frame with stamp: " <<
                        std::setprecision(15) <<
                        old_stamp.toSec() << 
                        " rn: " << ros::Time::now().toSec() <<
                        ", for seq nr: " << frame_seq <<
                        ", syncState: " << state_);

        if (state_ == SyncState::Uninitialised) {
            return false;
        }

        if (trigger_buffer_[channel].empty()) {
            return false;
        }

        const double kMaxExpectedDelay = 10e-3;
        const double age_cached_trigger = old_stamp.toSec() - trigger_buffer_[channel].rbegin()->second.toSec();
        
        if (std::fabs(age_cached_trigger) > kMaxExpectedDelay) {
            ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds: "
                                            << kMaxExpectedDelay << " seconds. Clearing trigger buffer...");
            frame_buffer_[channel].frame.reset();
            trigger_buffer_[channel].clear();
            state_ = wait_for_sync;
            return false;
        }

        if (state_ == wait_for_sync) {
            syncOffset(channel, frame_seq, old_stamp);
            return false;
        }

        uint32_t trigger_seq = frame_seq + sequence_offset_;
        auto it = trigger_buffer_[channel].find(trigger_seq);

        // if we haven't found a matching stamp
        if (it == trigger_buffer_[channel].end()) {
            // cached trigger is within the expected delay but does not match the expected seq nr
            // call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find trigger for seq: " <<  trigger_seq);
            syncOffset(channel, frame_seq, old_stamp);
            return false;
        }

        *new_stamp = it->second;
        *new_stamp = shiftTimestampToMidExposure(*new_stamp, exposure);
        trigger_buffer_[channel].clear();

        const double delay = age_cached_trigger;
        ROS_INFO_STREAM(log_prefix_ << "Matched frame to trigger: t" << trigger_seq << " -> c" << frame_seq <<
                        ", t_old " <<  std::setprecision(15) << old_stamp.toSec() << " -> t_new " << new_stamp->toSec() 
                        << std::setprecision(7) << " ~ " << delay);

        return true;
    }


    // Function to match an incoming hardware event (trigger/capture) to a buffered frame
    // Return true to publish frame, return false to buffer frame
    bool lookupFrame(const t_channel_id &channel, const uint32_t trigger_seq, 
                     ros::Time new_stamp, const ros::Time &old_stamp) {

        if (state_ == wait_for_sync) {
            return false; // do nothing if seq offset is not yet determined
        }

        uint32_t synced_seq = trigger_seq - sequence_offset_;
        if (frame_buffer_[channel].seq != synced_seq) {
            // cached frame is within the expected delay but does not match the expected seq nr
            // return false in order to call syncOffset()
            ROS_WARN_STREAM(log_prefix_ << "Could not find frame for seq: " << synced_seq);
            return  false;
        }

        if (!restamp_callback_) {
            ROS_WARN_STREAM_THROTTLE(10, log_prefix_ << " No callback set - discarding buffered images.");
            frame_buffer_[channel].frame.reset();
            return false;
        }

        // successfully matched frame to cached trigger
        new_stamp = shiftTimestampToMidExposure(new_stamp, frame_buffer_[channel].exposure);
        restamp_callback_(channel, new_stamp, frame_buffer_[channel].frame);
        
        // calc delay between mavros stamp and frame stamp
        const double delay = old_stamp.toSec() - new_stamp.toSec();
        ROS_INFO_STREAM(log_prefix_ << "Matched trigger to frame: t" << trigger_seq << " -> c" << synced_seq <<
                        ", t_old " <<  std::setprecision(15) << old_stamp.toSec() << " -> t_new " << new_stamp.toSec() 
                        << std::setprecision(7) << " ~ " << delay);

        geometry_msgs::PointStamped msg;
        msg.header.stamp = new_stamp;
        msg.point.x = delay;
        delay_pub_.publish(msg);
        frame_buffer_[channel].frame.reset();

        return true;
    }

    ros::Time shiftTimestampToMidExposure(const ros::Time &stamp, double exposure_us) {
        ros::Time new_stamp = stamp
                            + ros::Duration(exposure_us * 1e-6 / 2.0)
                            + ros::Duration(time_offset_ * 1e-3); // TODO check
        ROS_DEBUG_STREAM(log_prefix_ << "Shift timestamp: " << stamp.toSec() << " -> " << new_stamp.toSec() << " exposure: " << exposure_us * 1e-6);
        return new_stamp;
    }

    void cameraEventCallback(const mavros_msgs::CamIMUStamp &cam_imu_stamp) {

        if (state_ == SyncState::Uninitialised) {
            // Do nothing before setup and initialization
            ROS_ERROR("Received hardware events before init");
            return;
        }

        ROS_INFO_STREAM(log_prefix_ << "Received camera event stamp : " <<
                std::setprecision(15) <<
                cam_imu_stamp.frame_stamp.toSec() <<
                " rn: " << ros::Time::now().toSec() <<
                ", for seq nr: " << cam_imu_stamp.frame_seq_id <<
                " (synced_seq: " << cam_imu_stamp.frame_seq_id-sequence_offset_ << ")");

        for (auto channel : channel_set_) {

            if (!frame_buffer_[channel].frame) {
                // buffer stamp if there is no buffered frame
                trigger_buffer_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp;
                return;
            }

            const double kMaxExpectedDelay = 20e-3;
            const double age_cached_frame = cam_imu_stamp.frame_stamp.toSec() - frame_buffer_[channel].arrival_stamp.toSec();
            
            if (std::fabs(age_cached_frame) > kMaxExpectedDelay) {
                // buffered frame is too old. release buffered frame
                ROS_WARN_STREAM(log_prefix_ << "Delay out of bounds:  "
                                << kMaxExpectedDelay << " seconds. Releasing buffered frame...");
                frame_buffer_[channel].frame.reset();
                trigger_buffer_[channel].clear();
                trigger_buffer_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp;
                return;
            }

            if (!lookupFrame(channel, cam_imu_stamp.frame_seq_id, 
                             cam_imu_stamp.frame_stamp, frame_buffer_[channel].old_stamp)) {
                // lookupFrame() returns false:
                // waiting for sync or
                // OR 
                // seq numbers did not match: sync offsets
                trigger_buffer_[channel][cam_imu_stamp.frame_seq_id] = cam_imu_stamp.frame_stamp;
                syncOffset(channel, frame_buffer_[channel].seq, frame_buffer_[channel].old_stamp);
                return;
            }

            // synced: matched, re-stamped and published frame
        }
        return;
    }

 private:
    ros::NodeHandle nh_;

    const std::set<t_channel_id> channel_set_;

    int sequence_offset_;
    double time_offset_;

    ros::Subscriber camera_event_sub_;
    ros::Publisher delay_pub_;
    std::mutex mutex_;

    caching_callback restamp_callback_;

    // Synchronisation state
    SyncMode mode_;
    SyncState state_;

    //std::map<t_channel_id, std::string> logging_name_;

    // Synchronisation buffers
    std::map<t_channel_id, std::map<uint32_t, ros::Time>> trigger_buffer_;
    std::map<t_channel_id, frame_buffer_type> frame_buffer_;

    const std::string log_prefix_ = "[Mavros Triggering] ";
};

}

#endif //REALSENSE2_CAMERA_MAVROS_SYNCER_H
