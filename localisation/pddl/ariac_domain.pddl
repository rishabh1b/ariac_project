(define (domain bwdomain)
	(:requirements :strips :typing :fluents)
	(:types 
		Robot
		Bin
		Tray
		PartType)
	(:functions
		(current_quantity_in_tray ?part - PartType ?tray - Tray)
		(current_quantity_in_bin ?part - PartType ?bin - Bin)
		(order_parts ?part - PartType)
	)
	(:predicates
		(inbin ?part - PartType ?bin - Bin)
		(intray ?part - PartType ?tray - Tray)
		;;(on ?b - Block ?c - Block)
		;;(clear ?b - Block)
		(moverobot ?robot - Robot)
		(onbin ?robot - Robot ?bin - Bin)
		(ontray ?robot - Robot ?tray - Tray)
		(holding ?robot - Robot ?part - PartType)
		(handempty ?robot - Robot)
	)
     
	(:action move_over_bin
		:parameters(
			?robot - Robot
			?tray - Tray
			?bin - Bin)
		:precondition(and
			(ontray ?robot ?tray)
			(handempty ?robot))
		:effect(and
			(not(ontray ?robot ?tray))
			(moverobot ?robot)
			(onbin ?robot ?bin))
	)
	(:action move_over_tray
		:parameters(
			?robot - Robot
			?bin - Bin
			?part - PartType
			?tray - Tray)
		:precondition(and
			(onbin ?robot ?bin)
			(holding ?robot ?part))
		:effect(and
			(not(onbin ?robot ?bin))
			(moverobot ?robot)
			(ontray ?robot ?tray))
	)

	(:action pickup
		:parameters(
			?robot - Robot
			?part - PartType
			?bin - Bin)
		:precondition(and
			(>(current_quantity_in_bin ?part ?bin)0)
			(onbin ?robot ?bin)
			(inbin ?part ?bin)
			(handempty ?robot))
		:effect(and
			(holding ?robot ?part)
			(not(handempty ?robot))
			(decrease (current_quantity_in_bin ?part ?bin) 1))
	)

	(:action putdown
		:parameters(
			?robot - Robot
			?part - PartType
			?tray - Tray)
		:precondition(and
			(holding ?robot ?part)
			(ontray ?robot ?tray))
		:effect(and
			(not(holding ?robot ?part))
			(intray ?part ?tray)
			(handempty ?robot)
			(increase (current_quantity_in_tray ?part ?tray) 1))
	)
	;;(:action stack
	;;	:parameters(
	;;		?robot - Robot
	;;		?block1 - Block
	;;		?block2 - Block)
	;;	:precondition(and
	;;		(holding ?robot ?block1)
	;;		(clear ?block2))
	;;	:effect(and
	;;		(not(holding ?robot ?block1))
	;;		(not(clear ?block2))
	;;		(on ?block1 ?block2)
	;;		(clear ?block1)
	;;		(handempty ?robot))
	;;)
	;;(:action unstack
	;;	:parameters(
	;;		?robot - Robot
	;;		?block1 - Block
	;;		?block2 - Block)
	;;	:precondition(and
	;;		(on ?block1 ?block2)
	;;		(clear ?block1)
	;;		(handempty ?robot))
	;;	:effect(and
	;;		(not(on ?block1 ?block2))
	;;		(not(clear ?block1))
	;;		(not(handempty ?robot))
	;;		(holding ?robot ?block1)
	;;		(clear ?block2))
	;;)
)
