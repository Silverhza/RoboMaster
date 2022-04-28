
(cl:in-package :asdf)

(defsystem "modified_chassis-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MoveMode" :depends-on ("_package_MoveMode"))
    (:file "_package_MoveMode" :depends-on ("_package"))
  ))