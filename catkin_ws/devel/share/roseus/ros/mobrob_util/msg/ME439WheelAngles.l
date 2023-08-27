;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439WheelAngles)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439WheelAngles (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439WHEELANGLES")
  (make-package "MOBROB_UTIL::ME439WHEELANGLES"))

(in-package "ROS")
;;//! \htmlinclude ME439WheelAngles.msg.html


(defclass mobrob_util::ME439WheelAngles
  :super ros::object
  :slots (_ang_left _ang_right ))

(defmethod mobrob_util::ME439WheelAngles
  (:init
   (&key
    ((:ang_left __ang_left) 0.0)
    ((:ang_right __ang_right) 0.0)
    )
   (send-super :init)
   (setq _ang_left (float __ang_left))
   (setq _ang_right (float __ang_right))
   self)
  (:ang_left
   (&optional __ang_left)
   (if __ang_left (setq _ang_left __ang_left)) _ang_left)
  (:ang_right
   (&optional __ang_right)
   (if __ang_right (setq _ang_right __ang_right)) _ang_right)
  (:serialization-length
   ()
   (+
    ;; float64 _ang_left
    8
    ;; float64 _ang_right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _ang_left
       (sys::poke _ang_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ang_right
       (sys::poke _ang_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _ang_left
     (setq _ang_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ang_right
     (setq _ang_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mobrob_util::ME439WheelAngles :md5sum-) "8e9b84200a184649eafa090ea3ef1869")
(setf (get mobrob_util::ME439WheelAngles :datatype-) "mobrob_util/ME439WheelAngles")
(setf (get mobrob_util::ME439WheelAngles :definition-)
      "float64 ang_left
float64 ang_right
")



(provide :mobrob_util/ME439WheelAngles "8e9b84200a184649eafa090ea3ef1869")


