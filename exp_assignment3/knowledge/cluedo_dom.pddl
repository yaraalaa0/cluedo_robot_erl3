(define (domain cluedo_dom)

(:requirements :strips :typing :equality :fluents :disjunctive-preconditions :durative-actions)

(:types
    waypoint
)

(:predicates
    (base_at ?wp - waypoint)
    (unexplored ?wp - waypoint)
    (center ?wp - waypoint)
    (correct_hyp)
    
)

(:functions
    (hints)    ;;number of hints collected
)



(:action goto_waypointt
    :parameters (?from ?to - waypoint)
    :precondition (and (base_at ?from))
    :effect (and
        (base_at ?to)
        (not (base_at ?from))
    )
)


;;Receive a hint from the current location
(:action get_two_hint
    :parameters (?wp - waypoint)
    :precondition (and (base_at ?wp) (unexplored ?wp))
    :effect (and
        (increase (hints) 2.0)
        (not (unexplored ?wp))
    )
)



(:action check_hypothesis_correctt
    :parameters (?wp - waypoint)
    :precondition (and (>= (hints) 3.0) (center ?wp) (base_at ?wp) )
    :effect (and
        (correct_hyp)
        (assign (hints) 0.0)
    )
)



)
