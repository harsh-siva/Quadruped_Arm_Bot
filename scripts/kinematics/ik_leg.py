import numpy as np
from scipy.optimize import least_squares
from fk_leg import fk_leg, LEG_PARAMS

# How close (in meters) a solved foot position must land to the requested
# target for us to accept it as "reachable." 2mm is a placeholder — tune
# this down once you know the real hardware's positioning accuracy/backlash.
REACHABILITY_TOLERANCE_M = 0.002  # 2 mm

class UnreachableTargetError(Exception):
    """
    Raised by ik_leg() when the requested foot target is physically outside
    that leg's reachable workspace (i.e. no combination of joint angles,
    within their bounds, gets the foot close enough).

    Carries the BEST-EFFORT solution (closest_angles/closest_position) so
    the caller isn't left with nothing — if the caller wants a "get as
    close as possible" fallback instead of hard-failing, the data is right
    here on the exception object.
    """
    def __init__(self, leg, target, closest_angles, closest_position, error_m):
        self.leg = leg
        self.target = np.array(target)
        self.closest_angles = closest_angles
        self.closest_position = closest_position
        self.error_m = error_m
        super().__init__(
            f"[{leg}] Target {target} is unreachable — closest achievable position is "
            f"{closest_position}, off by {error_m*1000:.2f} mm."
        )

def ik_leg(leg, target_xyz, initial_guess=(0.0, 0.0, 0.0)):
    """
    Inverse kinematics: given a leg name and a desired foot position
    (target_xyz, relative to THAT LEG'S OWN coxa joint origin — same
    leg-local frame that fk_leg() returns), solve for the joint angles
    (theta_coxa, theta_femur, theta_tibia) that would place the foot there.

    This is the reverse problem of fk_leg(): fk_leg goes angles -> position,
    ik_leg goes position -> angles. There's no clean closed-form formula for
    this robot's geometry (each leg's axes are tilted differently), so
    instead we solve it NUMERICALLY:

      1. Start from a guess (initial_guess, default all zeros).
      2. Repeatedly try different angle combinations, each time checking
         "if I run FK on this guess, how far off is it from the target?"
         (that's exactly what residual() computes below).
      3. Keep adjusting the guess to shrink that error, respecting the
         leg's joint bounds the whole time, until the error can't be
         shrunk further (scipy's least_squares does this optimization
         loop for us).

    Returns: (angles, error_m) if the best solution found is within
    REACHABILITY_TOLERANCE_M of the target.
    Raises: UnreachableTargetError if even the best solution found is
    farther off than that tolerance — meaning the target is truly outside
    what this leg can physically reach.
    """
    p = LEG_PARAMS[leg]

    def residual(theta):
        # "How wrong is this guess?" — run FK on the guessed angles, and
        # return the vector difference from where we actually want the
        # foot to be. least_squares' whole job is to drive this vector
        # toward (0, 0, 0).
        foot = fk_leg(leg, *theta)
        return foot - np.array(target_xyz)

    result = least_squares(
        residual,
        x0=np.array(initial_guess),      # where the search starts
        bounds=(p["bounds_lower"], p["bounds_upper"]),  # never propose an
                                          # angle outside this leg's real
                                          # joint limits (per-leg, since
                                          # bounds differ across legs)
    )
    # result.x = the best angles least_squares could find.
    # Re-run FK on them (independent of the internal optimizer state) to
    # get the actual foot position that solution produces, and how far
    # that is from the target — this becomes our reachability check.
    closest_position = fk_leg(leg, *result.x)
    error = np.linalg.norm(residual(result.x))

    if error > REACHABILITY_TOLERANCE_M:
        raise UnreachableTargetError(leg, target_xyz, result.x, closest_position, error)

    return result.x, error

if __name__ == "__main__":
    # --- Round-trip test for all 4 legs ---
    # Pick a known set of joint angles, run FK to get the resulting foot
    # position (this becomes our "target"), then run IK on that target and
    # check whether IK's solved angles reproduce the SAME foot position via
    # FK again. This validates ik_leg() using fk_leg() as independent ground
    # truth — we're not checking IK against IK, we're checking it against
    # the already-verified forward kinematics.
    # (Note: the solved ANGLES don't need to exactly match test_angles —
    # some leg geometries could reach the same point via more than one
    # angle combination — what must match is the resulting foot POSITION.)
    test_angles = (0.3, -0.5, 0.9)

    for leg in LEG_PARAMS:
        print(f"=== Leg: {leg} ===")
        target = fk_leg(leg, *test_angles)
        print(f"Test angles:  {test_angles}")
        print(f"FK target:    {target}")

        solved_angles, err = ik_leg(leg, target, initial_guess=(0.0, 0.0, 0.0))
        print(f"IK solved:    {solved_angles}")
        print(f"Position error (m): {err:.8f}")

        check = fk_leg(leg, *solved_angles)
        print(f"FK of solved: {check}")
        match = np.allclose(check, target, atol=1e-6)
        print(f"Sub-micron round-trip match: {match}")
        print()

    # --- Unreachable target test ---
    # Deliberately request a foot position far outside any leg's physical
    # reach (0.9m straight out — the legs are ~0.23m long at full rest
    # distance, see fk_leg.py's "Distance from coxa origin" printout).
    # This confirms UnreachableTargetError actually fires when it should,
    # instead of silently returning a wildly wrong "solution."
    print("--- Unreachable target test (FR) ---")
    far_target = (0.9, 0.0, 0.0)
    try:
        angles, err = ik_leg("FR", far_target, initial_guess=(0.0, 0.0, 0.0))
        print(f"Unexpectedly reachable: {angles}, error {err}")
    except UnreachableTargetError as e:
        print(f"Correctly flagged as unreachable.")
        print(f"  Closest achievable position: {e.closest_position}")
        print(f"  Error: {e.error_m*1000:.2f} mm")
        print(f"  Fallback angles available at e.closest_angles: {e.closest_angles}")
