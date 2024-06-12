(define (domain vision)
    (:requirements :fluents :adl :typing)
    (:types carrot onion tomato - item - object)
    (:predicates 
        (has ?i - item)
        (offgrid ?i - item)
        (visible ?i - item)
    )
    (:functions 
        (xpos) (ypos) - integer
        (xloc ?o - object) (yloc ?o - object) - integer
        (walls) (viewdistance) - bit-matrix
    )
    (:action pickup
        :parameters (?i - item)
        :precondition (and (not (has ?i)) (= xpos (xloc ?i)) (= ypos (yloc ?i)))
        :effect (and (has ?i) (offgrid ?i)
                     (assign (xloc ?i) -10) (assign (yloc ?i) -10))
    )
    
    (:action up
        :precondition (and (> ypos 1)
                           (= (get-index walls (- ypos 1) xpos) false))
        :effect (and (decrease ypos 1)
                     (forall (?i - item)
                         (when (or (and (= (xloc ?i) xpos) 
                                        (or (= (yloc ?i) (+ ypos 3)) 
                                            (= (yloc ?i) (- ypos 3))))
                                   (and (= (yloc ?i) ypos) 
                                        (or (= (xloc ?i) (+ xpos 3)) 
                                            (= (xloc ?i) (- xpos 3)))))
                               (visible ?i)))))
    
    (:action down
        :precondition (and (< ypos (height walls))
                           (= (get-index walls (+ ypos 1) xpos) false))
        :effect (and (increase ypos 1)
                     (forall (?i - item)
                         (when (or (and (= (xloc ?i) xpos) 
                                        (or (= (yloc ?i) (+ ypos 3)) 
                                            (= (yloc ?i) (- ypos 3))))
                                   (and (= (yloc ?i) ypos) 
                                        (or (= (xloc ?i) (+ xpos 3)) 
                                            (= (xloc ?i) (- xpos 3)))))
                               (visible ?i)))))
    
    (:action left
        :precondition (and (> xpos 1)
                           (= (get-index walls ypos (- xpos 1)) false))
        :effect (and (decrease xpos 1)
                     (forall (?i - item)
                         (when (or (and (= (xloc ?i) xpos) 
                                        (or (= (yloc ?i) (+ ypos 3)) 
                                            (= (yloc ?i) (- ypos 3))))
                                   (and (= (yloc ?i) ypos) 
                                        (or (= (xloc ?i) (+ xpos 3)) 
                                            (= (xloc ?i) (- xpos 3)))))
                               (visible ?i)))))
    
    (:action right
        :precondition (and (< xpos (width walls))
                           (= (get-index walls ypos (+ xpos 1)) false))
        :effect (and (increase xpos 1)
                     (forall (?i - item)
                         (when (or (and (= (xloc ?i) xpos) 
                                        (or (= (yloc ?i) (+ ypos 3)) 
                                            (= (yloc ?i) (- ypos 3))))
                                   (and (= (yloc ?i) ypos) 
                                        (or (= (xloc ?i) (+ xpos 3)) 
                                            (= (xloc ?i) (- xpos 3)))))
                               (visible ?i)))))
)
