(define (problem vision-2)
    (:domain vision)
    (:objects
        carrot1 - carrot 
        onion1 - onion)
    (:init
        (= (xloc carrot1) 7)
        (= (yloc carrot1) 8)
        (= (xloc onion1) 5)
        (= (yloc onion1) 5)
        (= (walls) 
            (transpose (bit-mat 
                (bit-vec 0 0 0 0 0 0 0 0 0 0)
                (bit-vec 0 1 1 1 0 0 1 1 1 0)
                (bit-vec 0 1 0 0 0 1 0 0 1 0)
                (bit-vec 0 1 0 1 0 1 1 0 1 0)
                (bit-vec 0 0 0 1 0 0 0 0 0 0)
                (bit-vec 0 1 1 0 0 1 1 1 0 1)
                (bit-vec 0 1 0 0 1 0 0 1 0 1)
                (bit-vec 0 0 0 1 0 1 0 0 0 0)
                (bit-vec 0 1 0 0 0 0 1 1 0 1)
                (bit-vec 0 0 0 0 0 0 0 0 0 0)))
        )
        (= (xpos) 1)
        (= (ypos) 1)
    )
    (:goal (has carrot1))
)
