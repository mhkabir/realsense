#ifndef REALSENSE2_CAMERA_MAVROS_SYNCER_H
#define REALSENSE2_CAMERA_MAVROS_SYNCER_H

#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include <mavros_msgs/CommandTriggerInterval.h>
#include <mutex>
#include <tuple>

// Note on multi threading:
//      To avoid any confusion and non-defined state, this class locks a mutex for every function call
//      that is not const. This is due to the fact that many callbacks can happen simultaneously, especially
//      on callback based drivers such as the realsense. If not locked properly, this can lead to weird states.

namespace mavros_syncer {

template<typename t_channel_id>
class MavrosSyncer {
    // Callback definition for publishing synchronised frames
    typedef boost::function<void(const t_channel_id &channel,
                                 const ros::Time &stamp,
                                 const sensor_msgs::ImagePtr &frame,
                                 const sensor_msgs::CameraInfo &cinfo)> publish_callback;

    // Internal representation of a frame
    typedef struct {

        bool valid;

        // Frame sequence number from camera
        uint32_t seq;
        
        // Timestamp of frame (translated to ROS timebase)
        ros::Time frame_stamp;

        // Frame and metadata
        sensor_msgs::ImagePtr frame;
        sensor_msgs::CameraInfo cinfo;
        double exposure; // Exposure time in us

        void reset() {
            valid = false;
        }

    } frame_t;

    // Internal representation of a hardware event
    typedef struct {

        bool valid;

        // Frame sequence number from hardware
        uint32_t seq;
        
        // Timestamp of event (translated to ROS timebase)
        ros::Time event_stamp;

        void reset() {
            valid = false;
        }

    } event_t;

 public:

    MavrosSyncer(const std::set<t_channel_id> &channel_set) :
            _channel_set(channel_set),
            _state(SyncState::Uninitialised) {
        ROS_DEBUG_STREAM(_log_prefix << " Initialized with " << _channel_set.size() << " channels.");
        for (t_channel_id channel : _channel_set) {
            _event_buffer[channel].reset();
            _frame_buffer[channel].reset();
            _sequence_offset[channel] = 1;
        } // TODO : better init EVERYTHING with zeros in constructor list
    }

    void setup(const publish_callback &callback, int fps, double time_offset, int sync_mode) {

        _nh.setCallbackQueue(&_trigger_queue);

        _state = SyncState::Uninitialised;
        _mode = SyncMode(sync_mode);

        // _sequence_offset = 0;
        _time_offset = time_offset;

        _publish_callback = callback;

        _max_time_delta = 1.0/fps;

        _hardware_event_sub = _nh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 2,
                                     &MavrosSyncer::hardwareEventCallback, this, ros::TransportHints().udp());

        const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
        const std::string mavros_trig_interval_srv = "/mavros/cmd/trigger_interval";

        //if (_mode == SyncMode::Trigger) {
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
                /*
                mavros_msgs::CommandTriggerInterval req_interval;
                req_interval.request.cycle_time = 1000.0/fps;
                req_interval.request.integration_time = -1.0;
                ros::service::call(mavros_trig_interval_srv, req_interval);

                ROS_INFO("Set mavros trigger interval to %f! Success? %d Result? %d",
                                 1000.0/fps, req_interval.response.success, req_interval.response.result);*/
            } else {
                ROS_ERROR("Camera trigger setup services not available!");
            }
        //}
    }

    void start() {

        if (!_publish_callback) {
            ROS_ERROR_STREAM(_log_prefix << " No publish callback set - discarding buffered images.");
        }

        // Reset sequence number and enable triggering
        // _sequence_offset = 0; TODO
        /// if (_mode == SyncMode::Trigger) { 
            const std::string mavros_trig_control_srv = "/mavros/cmd/trigger_control";
            mavros_msgs::CommandTriggerControl req_enable;
            req_enable.request.trigger_enable = true;
            req_enable.request.sequence_reset = true;
            req_enable.request.trigger_pause = false;
            ros::service::call(mavros_trig_control_srv, req_enable);

            ROS_INFO_STREAM(_log_prefix << " Started triggering.");

            // TODO : needed?
            // Wait for the camera to align its exposure to hardware trigger
            // /ros::Duration(1.0).sleep();
        //}

        _trigger_queue.callAvailable(ros::WallDuration(0.9*_max_time_delta));

        _state = SyncState::WaitForSync;
    }

    bool channelValid(const t_channel_id &channel) const {
        return _channel_set.count(channel) == 1;
    }

    /*
    void bufferFrame(const t_channel_id &channel, const uint32_t seq, 
                     const ros::Time &frame_stamp, double exposure,
                     const sensor_msgs::ImagePtr frame, const sensor_msgs::CameraInfo cinfo) { // TODO : pass refs here, not copy

        if (!channelValid(channel)) {
            ROS_WARN_STREAM(_log_prefix << "bufferFrame called for invalid channel.");
            return;
        }

        if (_frame_buffer[channel].valid) {
            ROS_WARN_STREAM(_log_prefix << 
                "Overwriting image buffer! Make sure you're getting hardware events.");
        }

        // Buffer the frame
        _frame_buffer[channel].valid = true;
        _frame_buffer[channel].seq = seq;
        _frame_buffer[channel].frame_stamp = frame_stamp;
        _frame_buffer[channel].frame = frame;
        _frame_buffer[channel].cinfo = cinfo;
        _frame_buffer[channel].exposure = exposure;

        ROS_DEBUG_STREAM(_log_prefix << "Buffered frame, seq: " << seq);
    }*/

    void bufferEvent(const t_channel_id &channel, const uint32_t seq, 
                    const ros::Time &event_stamp) {

        if (!channelValid(channel)) {
            ROS_WARN_STREAM(_log_prefix << "bufferEvent called for invalid channel.");
            return;
        }

        const double event_age = ros::Time::now().toSec() - event_stamp.toSec();
        if (event_age < 0.0) {
            ROS_WARN_STREAM(_log_prefix << 
                    "Negative event age! Check time synchronisation.");
            return;
        }

        if (_event_buffer[channel].valid) {
            ROS_WARN_STREAM(_log_prefix << 
                    "Overwriting event buffer! Make sure you're getting frames.");
        }

        // Buffer the event
        _event_buffer[channel].valid = true;
        _event_buffer[channel].seq = seq;
        _event_buffer[channel].event_stamp = event_stamp;

         const double event_dt = event_stamp.toSec() - _last_event_stamp.toSec();
        _last_event_stamp = event_stamp;

        ROS_INFO_STREAM(_log_prefix << "Buffered event, seq: " << seq << " dt: " << event_dt << " age: " << event_age * 1000.0 << " ms");
    }

    void computeSequenceOffset(const t_channel_id &channel, const uint32_t event_seq, const uint32_t frame_seq, const double dt) {

        // Calculate offset between current hardware and camera sequence numbers
        _sequence_offset[channel] = int32_t(event_seq) - int32_t(frame_seq); // TODO : this has potential to overflow in the future

        // Between hardware stamp and camera stamp
        ROS_WARN_STREAM("Event age : " << dt * 1000.0 << " ms");

        /*
        if(fabs(dt) > 0.015) {
            ROS_ERROR("applying offset correction");
            _sequence_offset[channel]++;
        }*/

        ROS_INFO_STREAM(_log_prefix << 
                "New seq offset determined by channel " << channel.first << ": " << _sequence_offset[channel] << 
                ", from " << event_seq << " to " << frame_seq);

        _state = SyncState::Synchronised;

    }

    // Match an image frame to a buffered hardware event and publish it
    void matchEvent(const t_channel_id &channel, const uint32_t frame_seq,
                            const ros::Time &frame_stamp, double exposure, 
                            const sensor_msgs::ImagePtr img, const sensor_msgs::CameraInfo cinfo) {

        //if(frame_seq % 200 == 0) {
        //    ROS_ERROR("simulating frame drop");
        //    return;
        //}

        std::lock_guard<std::mutex> lg(_mutex);

        if (!channelValid(channel)) {
            ROS_WARN_STREAM(_log_prefix << "matchEvent called for invalid channel.");
            return;
        }

        if (_state == SyncState::Uninitialised) {
            ROS_ERROR("Uninitialised");
            return;
        }

        const double frame_dt = frame_stamp.toSec() - _last_frame_stamp.toSec();
        _last_frame_stamp = frame_stamp;

        ROS_INFO_STREAM(_log_prefix << std::setprecision(15) << 
                        "Received frame, dt: " << frame_dt * 1000.0 <<
                        ", seq : " << frame_seq);


        // Detect frame drops
        /*
        if(frame_seq != _last_frame_seq + 1 && frame_seq != 0) {
            ROS_WARN("Detected frame drop");
            // If the previous frame was dropped, 
            // then we do not have the corresponding event for this frame either. 

            // Abort and grab next event for next frame
            _trigger_queue.callOne();
            _trigger_queue.callOne(ros::WallDuration(0.8*_max_time_delta));
            _last_frame_seq = frame_seq;
            return;
        }
        _last_frame_seq = frame_seq;*/

        // Grab this event
        _trigger_queue.callOne(ros::WallDuration(0.8*_max_time_delta)); // TODO : this wait is bad?

        if (!_event_buffer[channel].valid) {
            ROS_ERROR("Event buffer empty");
            return;
        }
        
        // This should be approximately 2ms
        double event_age = ros::Time::now().toSec() - _event_buffer[channel].event_stamp.toSec();
        ROS_WARN("Event age at image callback : %f ms", event_age * 1000.0);

        if(event_age > 0.9*_max_time_delta) { 
            ROS_ERROR("Delay high, clear buffer : %f ms", event_age * 1000.0);
            //_trigger_queue.callAvailable();
            //getEvent(true, 0.8 * _max_time_delta);
            //_trigger_queue.callOne(ros::WallDuration(0.8*_max_time_delta)); // TODO : this doesn't actually wait if there is one in the buffer
            //_trigger_queue.callAvailable(ros::WallDuration(0.8 * _max_time_delta)); // TODO : this doesn't actually wait if there is one in the buffer
            //_trigger_queue.callOne();
            _trigger_queue.callAvailable(ros::WallDuration(0.8*_max_time_delta));
            _event_buffer[channel].reset();

            // known issue : sometimes 2 events arrive right after each other (previous one was delayed??)
            // and the second callOne gets it, overwriting the "needed" one. This then causes an incorrect sequence detection
            // Maybe we shouldn't try to compute here? Or maybe try to use the age and dt of events?
            // TODO : also a possibility when there is frame drop, but that is handled as a special case where there *should* 
            // be 2 events one after the other.
            _state = SyncState::WaitForSync;
            return;
        }

        /*
        if(event_age < 0.8*_max_time_delta) {
            ROS_ERROR("Received event early : %f ms", event_age * 1000.0);
            _state = SyncState::WaitForSync;
            return;
        }*/

        uint32_t expected_event_seq = frame_seq + _sequence_offset[channel]; // TODO : fix dis
        if (_state == SyncState::WaitForSync || _event_buffer[channel].seq != expected_event_seq) {
            if(_event_buffer[channel].seq != expected_event_seq) {
                ROS_ERROR_STREAM(_log_prefix << "expected: " << expected_event_seq << " in ev buffer: " << _event_buffer[channel].seq);
                computeSequenceOffset(channel, _event_buffer[channel].seq, frame_seq, event_age);
                _event_buffer[channel].reset();
                _trigger_queue.clear();
                //_trigger_queue.callOne(ros::WallDuration(0.8*_max_time_delta)); 
                return;
            }
            _state = SyncState::Synchronised;
        }

        // Successfully matched frame and event, publish it 
        // Correct timestamp for exposure time and static offset
        ros::Time corrected_stamp = correctTimestamp(_event_buffer[channel].event_stamp, 
                                                    exposure); // TODO make this more efficient

        _publish_callback(channel, corrected_stamp, img, cinfo);
    
        ROS_INFO_STREAM(_log_prefix << "frame#" << frame_seq << " -> stamp#" << expected_event_seq);

        // Clear event buffer after successful publish
        _event_buffer[channel].reset();

        // Grab next event
        //_trigger_queue.callOne(ros::WallDuration(0.8*_max_time_delta)); // TODO : needs to be re-thought for multiple channels
        //getEvent(false, 0.8 * _max_time_delta);
    }

    // Function to match an incoming hardware event (trigger/capture) to a buffered frame
    /*
    void matchFrame(const uint32_t event_seq, const ros::Time &event_stamp) {

        std::lock_guard<std::mutex> lg(_mutex);

        if (_state == SyncState::Uninitialised) {
            ROS_ERROR("Uninitialised");
            return;
        }

        const double event_dt = event_stamp.toSec() - _last_event_stamp.toSec();
        _last_event_stamp = event_stamp;

        // Match frame across all channels
        for (auto channel : _channel_set) {

            ROS_INFO_STREAM(_log_prefix << "Received event, dt : " <<
               event_dt <<
                ", with seq : " << event_seq <<
                " (synced_seq: " << event_seq - _sequence_offset[channel] << ")");

             if (!_frame_buffer[channel].valid) {
                // Empty frame buffer, buffer timestamp
                ROS_ERROR("frame buffer empty"); // expected
                bufferEvent(channel, event_seq, event_stamp);
                continue;
            }

            double sync_dt = _frame_buffer[channel].frame_stamp.toSec() - event_stamp.toSec();

            if(std::fabs(sync_dt) > 0.015) { 
                ROS_ERROR("sync dt high, reset");
                _state = SyncState::WaitForSync;
            }

            // Check for inter-event time deltas to detect drops
            if (std::fabs(event_dt) > _max_time_delta) {
                ROS_WARN_STREAM(_log_prefix << "Event delta out of bounds: "
                                                << event_dt << " seconds. Resetting synchroniser.");
                _state = SyncState::WaitForSync;
                
            }

            if (_state == SyncState::WaitForSync) {
                computeSequenceOffset(channel, event_seq, _frame_buffer[channel].seq, sync_dt);
                //computeSequenceOffset(channel, _event_buffer[channel].seq, frame_seq);
            }

            // TODO : only trust this if NOT WaitForSync
            uint32_t expected_frame_seq = event_seq - _sequence_offset[channel]; // TODO : fix dis
            if (_frame_buffer[channel].seq != expected_frame_seq) {
                ROS_ERROR_STREAM(_log_prefix << "expected: " << expected_frame_seq << " in frame buffer: " << _frame_buffer[channel].seq);
                //computeSequenceOffset(channel, frame_seq);
                continue;
            }

            ros::Time corrected_stamp = correctTimestamp(event_stamp, _frame_buffer[channel].exposure); // TODO make this more efficient

            _publish_callback(channel, corrected_stamp, _frame_buffer[channel].frame, _frame_buffer[channel].cinfo);

            ROS_INFO_STREAM(_log_prefix << "stamp#" << event_seq << " -> frame#" << expected_frame_seq);

            // Clear frame buffer after successful publish
            _frame_buffer[channel].reset();

        }

    }*/

    void getEvent(bool get_all, double wait_time) {

        if(ros::Time::now().toSec() - _last_event_stamp.toSec() < _max_time_delta) {
            // Already grabbed this event for a different channel
            ROS_WARN("already grabbed event");
            return;
        }

        if(get_all) {
            if(wait_time > 0.0) {
                //ros::Duration(wait_time).sleep();
                _trigger_queue.callAvailable(ros::WallDuration(wait_time));
            }
        } else {
            _trigger_queue.callOne(ros::WallDuration(wait_time));
        }
    }

    ros::Time correctTimestamp(const ros::Time &stamp, double exposure_us) {
        ros::Time new_stamp = stamp
                            - ros::Duration(exposure_us * 1e-6 / 2.0)   // Subtract half of exposure time
                            + ros::Duration(_time_offset);              // Add static time offset
        ROS_DEBUG_STREAM(_log_prefix << "Shift timestamp: " << stamp.toSec() << " -> " << new_stamp.toSec() << " exposure: " << exposure_us * 1e-6);
        return new_stamp;
    }

    void hardwareEventCallback(const mavros_msgs::CamIMUStamp &hardware_event) {

        //if(frame_seq % 300 == 0) {
        //    ROS_ERROR("simulating event drop");
        //    return;
        //}

        if (_state == SyncState::Uninitialised) {
            // Do nothing before setup and initialization
            ROS_ERROR("Received hardware events before init");
            return;
        }

        for (auto channel : _channel_set) {
            bufferEvent(channel, hardware_event.frame_seq_id, hardware_event.frame_stamp);
        }

        //matchFrame(hardware_event.frame_seq_id, hardware_event.frame_stamp);
    }

 private:
    ros::NodeHandle _nh;

    ros::CallbackQueue _trigger_queue;

    const std::set<t_channel_id> _channel_set;

    std::map<t_channel_id, int32_t> _sequence_offset;
    double _time_offset;

    double _max_time_delta;

    ros::Subscriber _hardware_event_sub;

    std::mutex _mutex;

    publish_callback _publish_callback;

    ros::Time _last_frame_stamp;
    ros::Time _last_event_stamp;
    uint32_t _last_frame_seq;
    uint32_t _last_event_seq;

    enum class SyncMode {
        None,
        Trigger,
        Capture,
    };

    // TODO
    enum class SyncState {
        Uninitialised,
        WaitForSync,        // Waiting to receive trigger/capture events
        Synchronised,       // Currently in synchronisation
    };

    // Synchronisation state
    SyncMode _mode;
    SyncState _state;

    // Synchronisation buffer
    std::map<t_channel_id, event_t> _event_buffer;
    std::map<t_channel_id, frame_t> _frame_buffer;

    const std::string _log_prefix = "[Hardware Sync] ";
};

}

#endif //REALSENSE2_CAMERA_MAVROS_SYNCER_H
