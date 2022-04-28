;; Auto-generated. Do not edit!


(when (boundp 'autofire::enemy_id)
  (if (not (find-package "AUTOFIRE"))
    (make-package "AUTOFIRE"))
  (shadow 'enemy_id (find-package "AUTOFIRE")))
(unless (find-package "AUTOFIRE::ENEMY_ID")
  (make-package "AUTOFIRE::ENEMY_ID"))

(in-package "ROS")
;;//! \htmlinclude enemy_id.msg.html


(defclass autofire::enemy_id
  :super ros::object
  :slots (_x _y _id ))

(defmethod autofire::enemy_id
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:id __id) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _id (string __id))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; string _id
    4 (length _id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get autofire::enemy_id :md5sum-) "7993d0d3043d5146dda81224aff3279c")
(setf (get autofire::enemy_id :datatype-) "autofire/enemy_id")
(setf (get autofire::enemy_id :definition-)
      "float32 x
float32 y
string id
")



(provide :autofire/enemy_id "7993d0d3043d5146dda81224aff3279c")


