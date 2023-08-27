;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439WheelDisplacements)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439WheelDisplacements (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439WHEELDISPLACEMENTS")
  (make-package "MOBROB_UTIL::ME439WHEELDISPLACEMENTS"))

(in-package "ROS")
;;//! \htmlinclude ME439WheelDisplacements.msg.html


(defclass mobrob_util::ME439WheelDisplacements
  :super ros::object
  :slots (_d_left _d_right ))

(defmethod mobrob_util::ME439WheelDisplacements
  (:init
   (&key
    ((:d_left __d_left) 0.0)
    ((:d_right __d_right) 0.0)
    )
   (send-super :init)
   (setq _d_left (float __d_left))
   (setq _d_right (float __d_right))
   self)
  (:d_left
   (&optional __d_left)
   (if __d_left (setq _d_left __d_left)) _d_left)
  (:d_right
   (&optional __d_right)
   (if __d_right (setq _d_right __d_right)) _d_right)
  (:serialization-length
   ()
   (+
    ;; float64 _d_left
    8
    ;; float64 _d_right
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _d_left
       (sys::poke _d_left (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _d_right
       (sys::poke _d_right (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _d_left
     (setq _d_left (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _d_right
     (setq _d_right (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get mobrob_util::ME439WheelDisplacements :md5sum-) "0c76547e5c694fd23ef9ed1874ecca4a")
(setf (get mobrob_util::ME439WheelDisplacements :datatype-) "mobrob_util/ME439WheelDisplacements")
(setf (get mobrob_util::ME439WheelDisplacements :definition-)
      "float64 d_left
float64 d_right
")



(provide :mobrob_util/ME439WheelDisplacements "0c76547e5c694fd23ef9ed1874ecca4a")


