
(cl:in-package :asdf)

(defsystem "pocketsphinx-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "string" :depends-on ("_package_string"))
    (:file "_package_string" :depends-on ("_package"))
  ))