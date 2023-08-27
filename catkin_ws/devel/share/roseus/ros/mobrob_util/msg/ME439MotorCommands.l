;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439MotorCommands)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439MotorCommands (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439MOTORCOMMANDS")
  (make-package "MOBROB_UTIL::ME439MOTORCOMMANDS"))

(in-package "ROS")
;;//! \htmlinclude ME439MotorCommands.msg.html


(defclass mobrob_util::ME439MotorCommands
  :super ros::object
  :slots (_cmd0 _cmd1 ))

(defmethod mobrob_util::ME439MotorCommands
  (:init
   (&key
    ((:cmd0 __cmd0) 0)
    ((:cmd1 __cmd1) 0)
    )
   (send-super :init)
   (setq _cmd0 (round __cmd0))
   (setq _cmd1 (round __cmd1))
   self)
  (:cmd0
   (&optional __cmd0)
   (if __cmd0 (setq _cmd0 __cmd0)) _cmd0)
  (:cmd1
   (&optional __cmd1)
   (if __cmd1 (setq _cmd1 __cmd1)) _cmd1)
  (:serialization-length
   ()
   (+
    ;; int16 _cmd0
    2
    ;; int16 _cmd1
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _cmd0
       (write-word _cmd0 s)
     ;; int16 _cmd1
       (write-word _cmd1 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _cmd0
     (setq _cmd0 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _cmd1
     (setq _cmd1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get mobrob_util::ME439MotorCommands :md5sum-) "853bb91b9c3f09306e8c71787dc35bdf")
(setf (get mobrob_util::ME439MotorCommands :datatype-) "mobrob_util/ME439MotorCommands")
(setf (get mobrob_util::ME439MotorCommands :definition-)
      "int16 cmd0
int16 cmd1
")



(provide :mobrob_util/ME439MotorCommands "853bb91b9c3f09306e8c71787dc35bdf")


