(define (domain knights-tour)
    (:requirements :strips :negative-preconditions)

    (:predicates
        (at ?col ?row)
        (visited ?col ?row)
        (diff_by_one ?x ?y)
        (diff_by_two ?x ?y)
    )

    (:action move_2col_1row
    :parameters (?from_col ?from_row ?to_col ?to_row)
    :precondition (and (at ?from_col ?from_row)
                        (diff_by_two ?from_col ?to_col)
                        (diff_by_one ?from_row ?to_row)
                        (not (visited ?to_col ?to_row))
                        )
    :effect (and (not (at ?from_col ?from_row))
                        (at ?to_col ?to_row)
                        (visited ?to_col ?to_row)
                        )
    )

    (:action move_2row_1col
        :parameters (?from_col ?from_row ?to_col ?to_row)
        :precondition (and (at ?from_col ?from_row)
                        (diff_by_two ?from_row ?to_row)
                        (diff_by_one ?from_col ?to_col)
                        (not (visited ?to_col ?to_row))
                        )
        :effect (and (not (at ?from_col ?from_row))
                        (at ?to_col ?to_row)
                        (visited ?to_col ?to_row)
                        )
    )
    
)