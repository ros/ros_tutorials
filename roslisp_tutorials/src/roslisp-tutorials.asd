;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem "roslisp-tutorials"
  :components
  ((:file "pkg")
   (:file "talker" :depends-on ("pkg"))
   (:file "listener" :depends-on ("pkg"))
   (:file "add-two-ints-client" :depends-on ("pkg"))
   (:file "add-two-ints-server" :depends-on ("pkg"))
   (:file "array-talker" :depends-on ("pkg"))
   (:file "array-listener" :depends-on ("pkg"))
   (:file "params" :depends-on ("pkg"))
   (:file "rosout-example" :depends-on ("pkg")))
	  
  :depends-on ("roslisp" "roslisp_tutorials-msg" "roslisp_tutorials-srv" "std_msgs-msg"))

;;;; eof
