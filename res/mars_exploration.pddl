(define (domain mars_exploration)
    (:requirements :strips :equality :typing)
    (:types rover Point sample)
    (:predicates
        (rover_at ?r ?x)
        (picture_taken ?x)
        (sample_from ?s ?x)
        (sample_taken ?s)
        (analyzed ?s)
        (communicated ?r ?x))

    (:action move
        :parameters (?r - rover ?x ?y - Point)
        :precondition (and (rover_at ?r ?x)
                           (not (= ?x ?y)))
        :effect (and (not (rover_at ?r ?x))
                     (rover_at ?r ?y)))

    (:action take_picture
        :parameters (?r - rover ?x - Point)
        :precondition (rover_at ?r ?x)
        :effect (picture_taken ?x))

    (:action drill
        :parameters (?r - rover ?x - Point ?s - sample)
        :precondition (and (sample_from ?s ?x)
                           (rover_at ?r ?x))
        :effect (sample_taken ?s))

    (:action communicate
        :parameters (?r - rover ?x - Point)
        :precondition (and (rover_at ?r ?x)
                           (not (communicated ?r ?x)))
        :effect (communicated ?r ?x))

    (:action analyse
        :parameters (?r - rover ?s - sample)
        :precondition (and (not (analyzed ?s))
                           (sample_taken ?s))
        :effect (analyzed ?s)))
