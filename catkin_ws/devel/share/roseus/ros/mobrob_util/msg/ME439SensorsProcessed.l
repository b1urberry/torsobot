;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439SensorsProcessed)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439SensorsProcessed (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439SENSORSPROCESSED")
  (make-package "MOBROB_UTIL::ME439SENSORSPROCESSED"))

(in-package "ROS")
;;//! \htmlinclude ME439SensorsProcessed.msg.html


(defclass mobrob_util::ME439SensorsProcessed
  :super ros::object
  :slots (_e0radians _e1radians _e0meters _e1meters _e0radpersec _e1radpersec _e0meterspersec _e1meterspersec _a0 _a1 _a2 _a3 _a4 _a5 _u0meters _u1meters _u2meters _dt ))

(defmethod mobrob_util::ME439SensorsProcessed
  (:init
   (&key
    ((:e0radians __e0radians) 0.0)
    ((:e1radians __e1radians) 0.0)
    ((:e0meters __e0meters) 0.0)
    ((:e1meters __e1meters) 0.0)
    ((:e0radpersec __e0radpersec) 0.0)
    ((:e1radpersec __e1radpersec) 0.0)
    ((:e0meterspersec __e0meterspersec) 0.0)
    ((:e1meterspersec __e1meterspersec) 0.0)
    ((:a0 __a0) 0.0)
    ((:a1 __a1) 0.0)
    ((:a2 __a2) 0.0)
    ((:a3 __a3) 0.0)
    ((:a4 __a4) 0.0)
    ((:a5 __a5) 0.0)
    ((:u0meters __u0meters) 0.0)
    ((:u1meters __u1meters) 0.0)
    ((:u2meters __u2meters) 0.0)
    ((:dt __dt) (instance ros::time :init))
    )
   (send-super :init)
   (setq _e0radians (float __e0radians))
   (setq _e1radians (float __e1radians))
   (setq _e0meters (float __e0meters))
   (setq _e1meters (float __e1meters))
   (setq _e0radpersec (float __e0radpersec))
   (setq _e1radpersec (float __e1radpersec))
   (setq _e0meterspersec (float __e0meterspersec))
   (setq _e1meterspersec (float __e1meterspersec))
   (setq _a0 (float __a0))
   (setq _a1 (float __a1))
   (setq _a2 (float __a2))
   (setq _a3 (float __a3))
   (setq _a4 (float __a4))
   (setq _a5 (float __a5))
   (setq _u0meters (float __u0meters))
   (setq _u1meters (float __u1meters))
   (setq _u2meters (float __u2meters))
   (setq _dt __dt)
   self)
  (:e0radians
   (&optional __e0radians)
   (if __e0radians (setq _e0radians __e0radians)) _e0radians)
  (:e1radians
   (&optional __e1radians)
   (if __e1radians (setq _e1radians __e1radians)) _e1radians)
  (:e0meters
   (&optional __e0meters)
   (if __e0meters (setq _e0meters __e0meters)) _e0meters)
  (:e1meters
   (&optional __e1meters)
   (if __e1meters (setq _e1meters __e1meters)) _e1meters)
  (:e0radpersec
   (&optional __e0radpersec)
   (if __e0radpersec (setq _e0radpersec __e0radpersec)) _e0radpersec)
  (:e1radpersec
   (&optional __e1radpersec)
   (if __e1radpersec (setq _e1radpersec __e1radpersec)) _e1radpersec)
  (:e0meterspersec
   (&optional __e0meterspersec)
   (if __e0meterspersec (setq _e0meterspersec __e0meterspersec)) _e0meterspersec)
  (:e1meterspersec
   (&optional __e1meterspersec)
   (if __e1meterspersec (setq _e1meterspersec __e1meterspersec)) _e1meterspersec)
  (:a0
   (&optional __a0)
   (if __a0 (setq _a0 __a0)) _a0)
  (:a1
   (&optional __a1)
   (if __a1 (setq _a1 __a1)) _a1)
  (:a2
   (&optional __a2)
   (if __a2 (setq _a2 __a2)) _a2)
  (:a3
   (&optional __a3)
   (if __a3 (setq _a3 __a3)) _a3)
  (:a4
   (&optional __a4)
   (if __a4 (setq _a4 __a4)) _a4)
  (:a5
   (&optional __a5)
   (if __a5 (setq _a5 __a5)) _a5)
  (:u0meters
   (&optional __u0meters)
   (if __u0meters (setq _u0meters __u0meters)) _u0meters)
  (:u1meters
   (&optional __u1meters)
   (if __u1meters (setq _u1meters __u1meters)) _u1meters)
  (:u2meters
   (&optional __u2meters)
   (if __u2meters (setq _u2meters __u2meters)) _u2meters)
  (:dt
   (&optional __dt)
   (if __dt (setq _dt __dt)) _dt)
  (:serialization-length
   ()
   (+
    ;; float32 _e0radians
    4
    ;; float32 _e1radians
    4
    ;; float32 _e0meters
    4
    ;; float32 _e1meters
    4
    ;; float32 _e0radpersec
    4
    ;; float32 _e1radpersec
    4
    ;; float32 _e0meterspersec
    4
    ;; float32 _e1meterspersec
    4
    ;; float32 _a0
    4
    ;; float32 _a1
    4
    ;; float32 _a2
    4
    ;; float32 _a3
    4
    ;; float32 _a4
    4
    ;; float32 _a5
    4
    ;; float32 _u0meters
    4
    ;; float32 _u1meters
    4
    ;; float32 _u2meters
    4
    ;; duration _dt
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _e0radians
       (sys::poke _e0radians (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e1radians
       (sys::poke _e1radians (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e0meters
       (sys::poke _e0meters (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e1meters
       (sys::poke _e1meters (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e0radpersec
       (sys::poke _e0radpersec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e1radpersec
       (sys::poke _e1radpersec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e0meterspersec
       (sys::poke _e0meterspersec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _e1meterspersec
       (sys::poke _e1meterspersec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a0
       (sys::poke _a0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a1
       (sys::poke _a1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a2
       (sys::poke _a2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a3
       (sys::poke _a3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a4
       (sys::poke _a4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _a5
       (sys::poke _a5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _u0meters
       (sys::poke _u0meters (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _u1meters
       (sys::poke _u1meters (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _u2meters
       (sys::poke _u2meters (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; duration _dt
       (write-long (send _dt :sec) s) (write-long (send _dt :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _e0radians
     (setq _e0radians (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e1radians
     (setq _e1radians (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e0meters
     (setq _e0meters (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e1meters
     (setq _e1meters (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e0radpersec
     (setq _e0radpersec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e1radpersec
     (setq _e1radpersec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e0meterspersec
     (setq _e0meterspersec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _e1meterspersec
     (setq _e1meterspersec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a0
     (setq _a0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a1
     (setq _a1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a2
     (setq _a2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a3
     (setq _a3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a4
     (setq _a4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _a5
     (setq _a5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _u0meters
     (setq _u0meters (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _u1meters
     (setq _u1meters (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _u2meters
     (setq _u2meters (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; duration _dt
     (send _dt :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _dt :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobrob_util::ME439SensorsProcessed :md5sum-) "44afd1f89023a5ad6799a84d47e3ad41")
(setf (get mobrob_util::ME439SensorsProcessed :datatype-) "mobrob_util/ME439SensorsProcessed")
(setf (get mobrob_util::ME439SensorsProcessed :definition-)
      "float32 e0radians
float32 e1radians
float32 e0meters
float32 e1meters
float32 e0radpersec
float32 e1radpersec
float32 e0meterspersec
float32 e1meterspersec
float32 a0
float32 a1
float32 a2
float32 a3
float32 a4
float32 a5
float32 u0meters
float32 u1meters
float32 u2meters
duration dt
")



(provide :mobrob_util/ME439SensorsProcessed "44afd1f89023a5ad6799a84d47e3ad41")


