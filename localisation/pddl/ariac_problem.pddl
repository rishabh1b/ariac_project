(define (problem bwproblem)
  (:domain bwdomain)
  ;;objects
  (:objects 
  	gear piston - PartType
  	robot - Robot
  	bin6 bin7 - Bin
    tray - Tray
  )

  ;;initial state
  (:init 
    (= (current_quantity_in_tray gear tray) 0)
    (= (current_quantity_in_tray piston tray) 0)
    (= (current_quantity_in_bin gear bin6) 10)
    (= (current_quantity_in_bin piston bin7) 10)
		(= (order_parts gear) 3)
		(= (order_parts piston) 2)
    (inbin gear bin6)
  	(inbin piston bin7)
    (onbin robot bin6)
  	(handempty robot)
  )

  ;;goal state
  (:goal (and
    (= (current_quantity_in_tray gear tray) (order_parts gear)) 
    (= (current_quantity_in_tray piston tray) (order_parts piston)))
  )
  )
