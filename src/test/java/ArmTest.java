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

    // Vertical Feed Forward should be 0.0
    assertEquals(0.0, arm.calcFeedForward(Constants.Arm.verticalCounts), TOLERANCE);

    // Horizontal Feed Forward should be gravityFeedForward
    assertEquals(
        Constants.Arm.gravityFeedForward,
        arm.calcFeedForward(Constants.Arm.horizontalCounts),
        TOLERANCE);

    // Test the arm clamp
    assertEquals(arm.calcClamp(0.5, 0.0, 1.0), 0.5, TOLERANCE);
    assertEquals(arm.calcClamp(1.5, 0.0, 1.0), 1.0, TOLERANCE);
    assertEquals(arm.calcClamp(-1.5, 0.0, 1.0), 0.0, TOLERANCE);

    // Test the arm rate limit
    assertEquals(arm.calcRateLimit(0.5, 0.0, 1.0), 0.5, TOLERANCE);
    assertEquals(arm.calcRateLimit(1.5, 0.0, 1.0), 1.0, TOLERANCE);
    assertEquals(arm.calcRateLimit(-1.5, 0.0, 1.0), -1.0, TOLERANCE);

    // Test the arm PID direction
    assertTrue(arm.calcPID(0.0, -90.0) < 0.0);
    assertTrue(arm.calcPID(-90.0, 0.0) > 0.0);
  }
}
