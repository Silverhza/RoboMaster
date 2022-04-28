;; Auto-generated. Do not edit!


(when (boundp 'yolo::Box)
  (if (not (find-package "YOLO"))
    (make-package "YOLO"))
  (shadow 'Box (find-package "YOLO")))
(unless (find-package "YOLO::BOX")
  (make-package "YOLO::BOX"))

(in-package "ROS")
;;//! \htmlinclude Box.msg.html


(defclass yolo::Box
  :super ros::object
  :slots (_xmin _ymin _xmax _ymax _confidence _obj _distance ))

(defmethod yolo::Box
  (:init
   (&key
    ((:xmin __xmin) 0.0)
    ((:ymin __ymin) 0.0)
    ((:xmax __xmax) 0.0)
    ((:ymax __ymax) 0.0)
    ((:confidence __confidence) 0.0)
    ((:obj __obj) "")
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _xmin (float __xmin))
   (setq _ymin (float __ymin))
   (setq _xmax (float __xmax))
   (setq _ymax (float __ymax))
   (setq _confidence (float __confidence))
   (setq _obj (string __obj))
   (setq _distance (float __distance))
   self)
  (:xmin
   (&optional __xmin)
   (if __xmin (setq _xmin __xmin)) _xmin)
  (:ymin
   (&optional __ymin)
   (if __ymin (setq _ymin __ymin)) _ymin)
  (:xmax
   (&optional __xmax)
   (if __xmax (setq _xmax __xmax)) _xmax)
  (:ymax
   (&optional __ymax)
   (if __ymax (setq _ymax __ymax)) _ymax)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:obj
   (&optional __obj)
   (if __obj (setq _obj __obj)) _obj)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float32 _xmin
    4
    ;; float32 _ymin
    4
    ;; float32 _xmax
    4
    ;; float32 _ymax
    4
    ;; float32 _confidence
    4
    ;; string _obj
    4 (length _obj)
    ;; float32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _xmin
       (sys::poke _xmin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ymin
       (sys::poke _ymin (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _xmax
       (sys::poke _xmax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ymax
       (sys::poke _ymax (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _obj
       (write-long (length _obj) s) (princ _obj s)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _xmin
     (setq _xmin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ymin
     (setq _ymin (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _xmax
     (setq _xmax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ymax
     (setq _ymax (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _obj
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _obj (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yolo::Box :md5sum-) "a88885b4a916fee405bb38cc0f4a8fce")
(setf (get yolo::Box :datatype-) "yolo/Box")
(setf (get yolo::Box :definition-)
      "float32 xmin # xmin
float32 ymin  # ymin
float32 xmax #xmax
float32 ymax #ymax

float32 confidence 
string obj
float32 distance

")



(provide :yolo/Box "a88885b4a916fee405bb38cc0f4a8fce")


