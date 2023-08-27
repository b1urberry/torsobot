;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439PathSpecs)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439PathSpecs (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439PATHSPECS")
  (make-package "MOBROB_UTIL::ME439PATHSPECS"))

(in-package "ROS")
;;//! \htmlinclude ME439PathSpecs.msg.html


(defclass mobrob_util::ME439PathSpecs
  :super ros::object
  :slots (_x0 _y0 _theta0 _Radius _Length ))

(defmethod mobrob_util::ME439PathSpecs
  (:init
   (&key
    ((:x0 __x0) 0.0)
    ((:y0 __y0) 0.0)
    ((:theta0 __theta0) 0.0)
    ((:Radius __Radius) 0.0)
    ((:Length __Length) 0.0)
    )
   (send-super :init)
   (setq _x0 (float __x0))
   (setq _y0 (float __y0))
   (setq _theta0 (float __theta0))
   (setq _Radius (float __Radius))
   (setq _Length (float __Length))
   self)
  (:x0
   (&optional __x0)
   (if __x0 (setq _x0 __x0)) _x0)
  (:y0
   (&optional __y0)
   (if __y0 (setq _y0 __y0)) _y0)
  (:theta0
   (&optional __theta0)
   (if __theta0 (setq _theta0 __theta0)) _theta0)
  (:Radius
   (&optional __Radius)
   (if __Radius (setq _Radius __Radius)) _Radius)
  (:Length
   (&optional __Length)
   (if __Length (setq _Length __Length)) _Length)
  (:serialization-length
   ()
   (+
    ;; float32 _x0
    4
    ;; float32 _y0
    4
    ;; float32 _theta0
    4
    ;; float32 _Radius
    4
    ;; float32 _Length
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x0
       (sys::poke _x0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y0
       (sys::poke _y0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta0
       (sys::poke _theta0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Radius
       (sys::poke _Radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Length
       (sys::poke _Length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x0
     (setq _x0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y0
     (setq _y0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta0
     (setq _theta0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Radius
     (setq _Radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Length
     (setq _Length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobrob_util::ME439PathSpecs :md5sum-) "278b13eb1b2cd154f8602d6118f2c62d")
(setf (get mobrob_util::ME439PathSpecs :datatype-) "mobrob_util/ME439PathSpecs")
(setf (get mobrob_util::ME439PathSpecs :definition-)
      "float32 x0
float32 y0
float32 theta0
float32 Radius
float32 Length
")



(provide :mobrob_util/ME439PathSpecs "278b13eb1b2cd154f8602d6118f2c62d")


