
(cl:in-package :asdf)

(defsystem "ros_yolo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArmorPoints" :depends-on ("_package_ArmorPoints"))
    (:file "_package_ArmorPoints" :depends-on ("_package"))
    (:file "Box" :depends-on ("_package_Box"))
    (:file "_package_Box" :depends-on ("_package"))
  ))