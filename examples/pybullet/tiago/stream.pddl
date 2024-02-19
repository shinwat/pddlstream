(define (stream tiago-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )
  (:stream sample-align
    :inputs (?o ?p1 ?p2)
    :domain (and (Alignable ?o) (Pose ?o ?p1) (Pose ?o ?p2))
    :outputs (?g)
    :certified (Alignment ?o ?p1 ?p2 ?g)
  )
  (:stream sample-hook
    :inputs (?o ?r ?p1 ?p2)
    :domain (and (Hookable ?o) (Pose ?o ?p1) (Pose ?o ?p2) (Tool ?r))
    :outputs (?p3 ?g)
    :certified (Hooked ?o ?p1 ?p2 ?p3 ?g)
  )
  (:stream plan-sweep-motion
    :inputs (?a ?o ?p1 ?p2 ?p3 ?g ?q1 ?q2);
    :domain (and (Controllable ?a) (Hooked ?o ?p1 ?p2 ?p3 ?g) (Pose ?o ?p1) (AConf ?a ?q2) (BConf ?q1))
    :outputs (?t)
    :certified (and (ATraj ?t) (ArmMotion ?a ?p1 ?p2 ?q1 ?q2 ?t))
  )
  (:stream inverse-hookable-kinematics; KLUDGE: copy IK stream because domain does not accept disjunction
    :inputs (?a ?r ?g1 ?o ?p1 ?p2 ?p3 ?g2)
    :domain (and (Controllable ?a) (Pose ?o ?p1) (Grasp ?r ?g1) (Hooked ?o ?p1 ?p2 ?p3 ?g2))
    :outputs (?q ?q2 ?t)
    :certified (and (BConf ?q) (ATraj ?t) (AConf ?a ?q2) (Kin3 ?a ?r ?p1 ?p2 ?g2 ?q ?q2 ?t));
  )
  (:stream plan-push-motion
    :inputs (?a ?o ?p1 ?p2 ?g ?q1 ?q2)
    :domain (and (Controllable ?a) (Alignment ?o ?p1 ?p2 ?g) (Pose ?o ?p1) (AConf ?a ?q2) (BConf ?q1))
    :outputs (?t)
    :certified (and (ATraj ?t) (ArmMotion ?a ?p1 ?p2 ?q1 ?q2 ?t))
  )
  (:stream inverse-reachable-kinematics; KLUDGE: copy IK stream because domain does not accept disjunction
    :inputs (?a ?o ?p1 ?p2 ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p1) (Alignment ?o ?p1 ?p2 ?g)); (Pose ?o ?p2)
    :outputs (?q1 ?q2 ?t)
    :certified (and (BConf ?q1) (AConf ?a ?q2) (ATraj ?t) (Kin2 ?a ?o ?p1 ?p2 ?g ?q1 ?q2 ?t));  (Kin2 ?a ?o ?p ?g ?q1 ?q2 ?t)
  )
  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g))
    :outputs (?q ?t)
    :certified (and (BConf ?q) (ATraj ?t) (Kin ?a ?o ?p ?g ?q ?t))
  )
  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    ;:fluents (AtPose AtGrasp) ; AtBConf
    :outputs (?t)
    :certified (and (BTraj ?t) (BaseMotion ?q1 ?t ?q2))
  )

  (:stream test-cfree-pose-pose
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?o1 ?p1 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeApproachPose ?o1 ?p1 ?g1 ?o2 ?p2)
  )
  (:stream test-cfree-align-pose
    :inputs (?o1 ?p1 ?p3 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Alignment ?o1 ?p1 ?p3 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeAlignPose ?o1 ?p1 ?p3 ?g1 ?o2 ?p2)
  )
  (:stream test-cfree-traj-pose
    :inputs (?t ?o2 ?p2)
    :domain (and (ATraj ?t) (Pose ?o2 ?p2))
    :certified (CFreeTrajPose ?t ?o2 ?p2)
  )
  ;(:stream test-cfree-traj-grasp-pose
  ;  :inputs (?t ?a ?o1 ?g1 ?o2 ?p2)
  ;  :domain (and (BTraj ?t) (Arm ?a) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
  ;  :certified (CFreeTrajGraspPose ?t ?a ?o1 ?g1 ?o2 ?p2)
  ;)

  (:function (Distance ?q1 ?q2)
    (and (BConf ?q1) (BConf ?q2))
  )
  ; (:function (MoveCost ?t)
  ;  (and (BTraj ?t))
  ; )
)