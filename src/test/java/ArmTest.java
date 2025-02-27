package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ArmTest {

  private static final double TOLERANCE = 1e-6;
  private Arm arm;

  @BeforeEach
  public void setUp() {
    arm = new Arm();
  }

  @AfterEach
  public void tearDown() {
    arm = null;
  }

  @Test
  public void testArmFunctions() {

    // Test the arm rate limit (no limiting, clip high, clip low)
    assertEquals(arm.calcRateLimit(37.5, 37.0, 1.0), 37.5, TOLERANCE);
    assertEquals(arm.calcRateLimit(99.0, 37.0, 1.0), 38.0, TOLERANCE);
    assertEquals(arm.calcRateLimit(-99.0, 37.0, 1.0), 36.0, TOLERANCE);

    // Test the arm clamp (no clamping, clip high, clip low)
    assertEquals(arm.calcClamp(24.0, -37.0, 37.0), 24.0, TOLERANCE);
    assertEquals(arm.calcClamp(99.0, -37.0, 37.0), 37.0, TOLERANCE);
    assertEquals(arm.calcClamp(-99.0, -37.0, 37.0), -37.0, TOLERANCE);

    // Test the arm PID direction (going negative should give negative output)
    assertTrue(arm.calcPID(0.0, -90.0) < 0.0);
    assertTrue(arm.calcPID(-90.0, 0.0) > 0.0);

    // Vertical Feed Forward should be 0.0
    assertEquals(0.0, arm.calcFeedForward(Constants.Arm.verticalCounts), TOLERANCE);

    // Horizontal Feed Forward should be gravityFeedForward
    assertEquals(
        Constants.Arm.gravityFeedForward,
        arm.calcFeedForward(Constants.Arm.horizontalCounts),
        TOLERANCE);
  }
}
