;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :robots-common)

(defun current-robot-symbol ()
  (let ((robot-symbol
          (cut:var-value '?r (car (prolog:prolog '(cram-robot-interfaces:robot ?r))))))
    (if (cut:is-var robot-symbol)
        NIL
        robot-symbol)))

(defun current-robot-package ()
  (symbol-package (current-robot-symbol)))

(defun current-robot-name ()
  (symbol-name (current-robot-symbol)))


(defun try-reference-designator-json (designator-json-string)
  (let (success designator grounding)
    (handler-case
        (progn (roslisp:ros-info
                (robots-common reference-server)
                "Designator resolved to: ~a"
                (setf grounding (desig:reference
                                 (setf designator (parse-action-json designator-json-string)))))
               (setf success T))

      (desig:designator-error (error-object)
        (roslisp:ros-error (robots-common reference-server)
                           "Could not resolve designator: ~a"
                           error-object))

      (json-parser-failed (error-object)
        (roslisp:ros-error (robots-common reference-server)
                           "Could not parse JSON designator: ~a"
                           error-object))

      (error (error-object)
        (roslisp:ros-error (robots-common reference-server)
                           "Generic error happened: ~a.~%Please handle properly." error-object)))
    (values success designator grounding)))


(defun action-type-class-name (action-type)
  (declare (type string action-type))
  "partly taken from roslisp implementation of MAKE-MESSAGE-FN"
  (destructuring-bind (pkg-name type)
      (roslisp-utils:tokens (string-upcase (actionlib::action-msg-type action-type "Goal"))
                            :separators '(#\/))
    (let ((pkg (find-package (intern (concatenate 'string pkg-name "-MSG") 'keyword))))
      (assert pkg nil "Can't find package ~a-MSG" pkg-name)
      (let ((class-name (find-symbol type pkg)))
        (assert class-name nil "Can't find class for ~a" action-type)
        class-name))))


(defun rosify (lispy-symbol)
  (declare (type symbol lispy-symbol))
  "taken from cram_json_prolog PROLOGIFY function"
  (flet ((contains-lower-case-char (symbol)
           (and
            (find-if (lambda (ch)
                       (let ((lch (char-downcase ch)))
                         (and (find lch "abcdefghijklmnopqrstuvwxyz")
                              (eq lch ch))))
                     (symbol-name symbol))
            t)))
    (if (contains-lower-case-char lispy-symbol)
        (string lispy-symbol)
        (string-downcase (substitute #\_ #\- (copy-seq (string lispy-symbol)))))))

(defun derosify (ros-string &optional package)
  (declare (type string ros-string))
  (intern (string-upcase (substitute #\- #\_ ros-string))
          (or package :keyword)))


(defmacro make-symbol-type-message (msg-type-as-symbol &rest args)
  "Same as roslisp:make-message only better"
  `(roslisp::set-fields-fn
    (make-instance ,msg-type-as-symbol)
    ,@(loop
        for i from 0
        for arg in args
        collect (if (evenp i)
                    `',(mapcar
                        #'roslisp::convert-to-keyword
                        (roslisp::designated-list arg))
                    arg))))


(def-fact-group sherpa-utils ()

  (<- (terrain-name terrain))

  (<- (location-pose ?location ?pose)
    (-> (lisp-type ?location designator)
        ;; (desig-reference ?location ?pose)
        (and (desig-solutions ?location ?poses)
             (member ?pose ?poses))
        (or (cram-tf:pose ?pose ?location)
            (equal ?location ?pose)))))
