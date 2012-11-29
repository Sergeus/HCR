
(cl:in-package :asdf)

(defsystem "ros_speech_engine-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "string" :depends-on ("_package_string"))
    (:file "_package_string" :depends-on ("_package"))
  ))