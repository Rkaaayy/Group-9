(define (domain ev_charging)
  (:requirements :strips :negative-preconditions :action-costs)

  ;; ──────────── predicates ────────────
  (:predicates
    (charging)
    (temperature_high)          ;; true when battery is above safe limit

    (motor_off)
    (motor_low)
    (motor_medium)
    (motor_full)

    (ac_off)
    (ac_on)

    (infotainment_off)
    (infotainment_on)
  )

  (:functions (total-cost))

  ;; ──────────── motor actions ────────────
  (:action set_motor_off
    :precondition (and (not (motor_off)))
    :effect (and
      (motor_off)
      (not (motor_low)) (not (motor_medium)) (not (motor_full))
      (increase (total-cost) 1))
  )

  (:action set_motor_low
    :precondition (and (charging) (not (temperature_high)) (motor_off))
    :effect (and
      (motor_low)
      (not (motor_off))
      (increase (total-cost) 2))
  )

  (:action set_motor_medium
    :precondition (and (charging) (not (temperature_high)) (motor_low))
    :effect (and
      (motor_medium)
      (not (motor_low))
      (increase (total-cost) 3))
  )

  (:action set_motor_full
    :precondition (and (charging) (not (temperature_high)) (motor_medium))
    :effect (and
      (motor_full)
      (not (motor_medium))
      (increase (total-cost) 4))
  )

  ;; ──────────── AC actions ────────────
  (:action set_ac_on
    :precondition (ac_off)
    :effect (and (ac_on) (not (ac_off)) (increase (total-cost) 1))
  )

  (:action set_ac_off
    :precondition (ac_on)
    :effect (and (ac_off) (not (ac_on)) (increase (total-cost) 1))
  )

  ;; ──────────── infotainment actions ────────────
  (:action set_infotainment_on
    :precondition (infotainment_off)
    :effect (and (infotainment_on) (not (infotainment_off)) (increase (total-cost) 1))
  )

  (:action set_infotainment_off
    :precondition (infotainment_on)
    :effect (and (infotainment_off) (not (infotainment_on)) (increase (total-cost) 1))
  )
)

