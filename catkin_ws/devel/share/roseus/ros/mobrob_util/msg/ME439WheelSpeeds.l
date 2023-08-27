;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439WheelSpeeds)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439WheelSpeeds (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439WHEELSPEEDS")
  (make-package "MOBROB_UTIL::ME439WHEELSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude ME439WheelSpeeds.msg.html


(defclass mobrob_util::ME439WheelSpeeds
  :super ros::object
  :slots (_v_left _v_right ))

(defmethod mobrob_util::ME439WheelSpeeds
  (:init
   (&key
    ((:v_left __v_left) 0.0)
    ((:v_right __v_right) 0.0)
    )
   (send-super :init)
   (setq _v_left (float __v_left))
   (setq _v_right (float __v_right))
   self)
  (:v_left
   (&optional __v_left)
   (if __v_left (setq _v_left __v_left)) _v_left)
  (:v_right
   (&optional __v_right)
   (if __v_right (setq _v_right __v_right)) _v_right)
  (:serialization-length
   ()
   (+
    ;; float32 _v_left
    4
    ;; float32 _v_right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _v_left
       (sys::poke _v_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v_right
       (sys::poke _v_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _v_left
     (setq _v_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v_right
     (setq _v_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobrob_util::ME439WheelSpeeds :md5sum-) "db3ff789940b54ebf58420f0dabd5157")
(setf (get mobrob_util::ME439WheelSpeeds :datatype-) "mobrob_util/ME439WheelSpeeds")
(setf (get mobrob_util::ME439WheelSpeeds :definition-)
      "float32 v_left
float32 v_right
")



(provide :mobrob_util/ME439WheelSpeeds "db3ff789940b54ebf58420f0dabd5157")


