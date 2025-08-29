(define (problem turn_it_off)
    (:domain switch)

    (:init
        (switch_is_on) 
    )

    (:goal 
        (switch_is_off)
    )
)