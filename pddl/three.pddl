(define (problem three)
    (:domain philosophers)

    (:objects
        Kant Heidegger Wittgenstein - philosopher
        f1 f2 f3 - fork
    )

    (:init
        (is_right_fork Kant f1)
        (is_right_fork Heidegger f2)
        (is_right_fork Wittgenstein f3)
        (is_left_fork Kant f3)
        (is_left_fork Heidegger f1)
        (is_left_fork Wittgenstein f2)
        (thinking Kant)
        (thinking Heidegger)
        (thinking Wittgenstein)
        (free f1)
        (free f2)
        (free f3)
        (normal)
    )

    (:goal (and
        (deadlocked Kant)
        (deadlocked Heidegger)
        (deadlocked Wittgenstein)
    )
    )

)