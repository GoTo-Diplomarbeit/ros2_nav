# Project Idea

GoTo is a mobile robot that guides people through a building to a room they choose. It maps the building on its own, then uses that map to lead a person to their destination. A Bluetooth connection to the user's phone keeps the robot and the person together along the way.

## 1. Autonomous Mapping with User Override

- The robot explores and maps the **entire building on its own** (SLAM and frontier-based exploration).
- At any time the user can **override** exploration, for example to:
  - pause, resume or stop exploration,
  - drive the robot manually to areas it missed or should avoid,
  - correct or confirm what the robot has mapped.
- The finished map is saved and annotated with **rooms**, which become the possible destinations.

## 2. Guiding a Person to a Room

- The saved map is used to plan a path from the robot's current position to the selected room.
- The robot drives ahead of the person and leads them along that path.

## 3. Website with Bluetooth Connection

- A website (on the user's smartphone) **connects to the robot over Bluetooth**.
- Once connected, the user **selects the room** they want to go to.
- The website then starts the guidance and shows its status for the rest of the trip.

## 4. Detecting When the User Falls Behind

- The robot continuously measures the **Bluetooth signal strength (RSSI)** of the user's device.
- If the signal drops below a threshold, the user is assumed to be **falling behind**. The robot then:
  - **stops** and waits,
  - **informs the user on the website** that they are falling behind,
  - shows the **direction in which the robot is**, based on where it was when the signal became too weak.
- Guidance continues once the signal is strong enough again.

## 5. Waiting at Corners

- At every **corner or turn** on the path, the robot **stops and waits**.
- It only continues once the user is **within a set radius**, which it detects from a high enough Bluetooth signal strength.
- This way the user never loses sight of the robot when it changes direction.

## 6. Returning to the Start

- Once the destination room is reached, the guidance is finished and the website shows that the user has arrived.
- The robot then **drives back to its starting location** on its own, ready for the next user.

## 7. Direction Arrow on the Website

- During guidance, the website shows an **arrow pointing in the direction the user should go**.
- The arrow updates as the robot moves, at corners, and when the user has fallen behind (then pointing towards the robot).

## Overall Flow

1. Robot maps the building on its own, with optional user override, and rooms are defined on the map.
2. The user opens the website and connects to the robot via Bluetooth.
3. The user selects a destination room.
4. The robot guides the user, while the website shows a direction arrow:
   - it waits at corners until the user is close enough,
   - it stops and warns the user (with the robot's direction) if they fall behind.
5. The robot reaches the room and the website confirms arrival.
6. The robot returns to its starting location.
