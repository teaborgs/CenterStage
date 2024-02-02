package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

/**
 * A class that simplifies input handling for gamepads
 * Uses reflection to get the state of the gamepad buttons and axes
 * <p>
 * Example usage:
 * <pre>
 *         InputSystem input = new InputSystem(gamepad1);
 *         if (input.wasPressedThisFrame(new InputSystem.Key("a"))) {
 *            // do something only the first frame A is pressed down
 *         }
 *         if (input.isPressed(new InputSystem.Key("b"))) {
 *            // do something while B is pressed down
 *         }
 *         if (input.wasPressedThisFrame(new InputSystem.BindingCombo("combo", new InputSystem.Key("a"), new InputSystem.Key("b")))) {
 *            // do something only the first frame A and B are pressed down
 *         }
 *         if (input.isPressed(new InputSystem.BindingCombo("combo", new InputSystem.Key("a"), new InputSystem.Key("b")))) {
 *            // do something while A and B are pressed down
 *         }
 *         if (input.getValue(new InputSystem.Axis("left_stick_x")) > 0.5) {
 *            // do something while the left stick is pushed to the right more than 50%
 *         }
 *     </pre>
 * <p>
 * The gamepad buttons and axes are represented by the following classes:
 * <ul>
 *     <li>{@link Key} - Represents a button on the gamepad</li>
 *     <li>{@link Axis} - Represents an axis on the gamepad</li>
 *     <li>{@link BindingCombo} - Represents a combination of buttons and axes on the gamepad</li>
 *     <li>{@link Binding} - Base class for all the above classes</li>
 *     <li>{@link Gamepad} - The gamepad class</li>
 * </ul>
 */
public class InputSystem
{
	private final Gamepad gamepad;
	private final Class<? extends Gamepad> gamepadClass;
	private final Telemetry telemetry;
	private final HashMap<Key, Boolean> keyStates = new HashMap<>();

	public InputSystem(Gamepad gamepad)
	{
		this(gamepad, null);
	}

	public InputSystem(Gamepad gamepad, Telemetry telemetry)
	{
		this.gamepad = gamepad;
		this.gamepadClass = gamepad.getClass();
		this.telemetry = telemetry;
	}

	public boolean wasPressedThisFrame(Key key)
	{
		try {
			boolean currentState = (boolean) gamepadClass.getField(key.getId()).getBoolean(gamepad);
			if (!keyStates.containsKey(key)) keyStates.put(key, currentState);
			else {
				Boolean state = keyStates.get(key);
				if (currentState && Boolean.FALSE.equals(state)) keyStates.put(key, true);
				else if (currentState && Boolean.TRUE.equals(state)) return false;
				else if (!currentState || Boolean.TRUE.equals(state)) keyStates.put(key, false);
			}
			return Boolean.TRUE.equals(keyStates.get(key));
		} catch (Exception e) {
			print("[ERROR] Could not get key " + key.getId());
			return false;
		}
	}

	public boolean wasPressedThisFrame(BindingCombo keyCombo)
	{
		for (Binding key : keyCombo.getBindings())
			if (key instanceof Key && !wasPressedThisFrame((Key) key)) return false;
			else if (key instanceof Axis && getValue((Axis) key) <= 0.75) return false;
		return true;
	}

	public boolean isPressed(Key key)
	{
		try {
			return (boolean) gamepadClass.getField(key.getId()).getBoolean(gamepad);
		} catch (Exception e) {
			print("[ERROR] Could not get key " + key.getId());
			return false;
		}
	}

	public boolean isPressed(BindingCombo keyCombo)
	{
		for (Binding key : keyCombo.getBindings())
			if (key instanceof Key && !isPressed((Key) key)) return false;
			else if (key instanceof Axis && getValue((Axis) key) <= 0.75) return false;
		return true;
	}

	public double getValue(Axis key)
	{
		try {
			return (double) gamepadClass.getField(key.getId()).getDouble(gamepad);
		} catch (Exception e) {
			print("[ERROR] Could not get key " + key.getId());
			return 0d;
		}
	}

	private void print(String message)
	{
		if (telemetry == null) return;
		telemetry.addLine("[InputSystem] " + message);
		telemetry.update();
	}

	public static abstract class Binding
	{
		private final String id;

		public Binding(String id)
		{
			this.id = id;
		}

		public String getId()
		{
			return id;
		}
	}

	public static class Key extends Binding
	{
		public Key(String id)
		{
			super(id);
		}
	}

	public static class Axis extends Binding
	{
		public Axis(String id)
		{
			super(id);
		}
	}

	public static class BindingCombo extends Binding
	{
		private final Binding[] bindings;

		public BindingCombo(String id, Binding... bindings)
		{
			super(id);
			this.bindings = bindings;
			if (bindings.length == 0)
				throw new IllegalArgumentException("BindingCombo must have at least one binding");
			for (Binding binding : bindings)
				if (binding instanceof BindingCombo)
					throw new IllegalArgumentException("BindingCombo cannot contain BindingCombos");
		}

		public Binding[] getBindings()
		{
			return bindings;
		}
	}
}