(define (domain ev_charging)
  (:requirements :strips :negative-preconditions :typing :action-costs)

  ;; ──────────── PREDICATES ────────────
  (:predicates
    ;; Original predicates
    (charging)
    (temperature_high)
    (motor_off)
    (motor_low)
    (motor_medium)
    (motor_full)
    (ac_off)
    (ac_on)
    (infotainment_off)
    (infotainment_on)
    
    ;; New battery state predicates
    (battery_extremely_low)
    (battery_low)
    (battery_medium)
    (battery_high)
    (battery_full)
  )

  (:functions (total-cost))

  ;; ──────────── MOTOR ACTIONS ────────────
  (:action set_motor_off
    :precondition (not (motor_off))
    :effect (and
      (motor_off)
      (not (motor_low)) (not (motor_medium)) (not (motor_full))
      (increase (total-cost) 1))
  )

  (:action set_motor_low
    :precondition (and 
      (charging)
      (not (temperature_high))
      (or (battery_low) (battery_medium) (battery_high) (battery_full))
    )
    :effect (and
      (motor_low)
      (not (motor_off))
      (increase (total-cost) 4))  ;; Higher cost to discourage when battery is high
  )

  (:action set_motor_medium
    :precondition (and 
      (charging)
      (not (temperature_high))
      (or (battery_medium) (battery_high) (battery_full))
    )
    :effect (and
      (motor_medium)
      (not (motor_low))
      (increase (total-cost) 3))
  )

  (:action set_motor_full
    :precondition (and 
      (charging)
      (not (temperature_high))
      (or (battery_high) (battery_full))
    )
    :effect (and
      (motor_full)
      (not (motor_medium))
      (increase (total-cost) 1))  ;; Cheapest when battery is high
  )

  ;; ──────────── AC ACTIONS ────────────
  (:action set_ac_on
    :precondition (and 
      (ac_off)
      (not (battery_extremely_low))
      (not (battery_low))
    )
    :effect (and 
      (ac_on) 
      (not (ac_off))
      (increase (total-cost) 2))
  )

  (:action set_ac_off
    :precondition (ac_on)
    :effect (and 
      (ac_off) 
      (not (ac_on))
      (increase (total-cost) 1))
  )

  ;; ──────────── INFOTAINMENT ACTIONS ────────────
  (:action set_infotainment_on
    :precondition (and 
      (infotainment_off)
      (not (battery_extremely_low))
    )
    :effect (and 
      (infotainment_on) 
      (not (infotainment_off))
      (increase (total-cost) 1))
  )

  (:action set_infotainment_off
    :precondition (infotainment_on)
    :effect (and 
      (infotainment_off) 
      (not (infotainment_on))
      (increase (total-cost) 1))
  )
)
