(input 03_height_C_H.txt file)
# Issues

### Initial Condition
#### Opacity Map -> Grid Map
##### Character
- 0 : empty
- n(positive integer) : height of obstacle
- C : Current Car
- H : Human
  
# Instructions
### Objective

#### Never miss any sentence with the `<!>` tag.
- The tag will appear at the beginning of the sentence.
- Double-check to ensure no tags are missed.
#### Reading and Displaying the Grid Map 
- Draw the map using **matplotlib**, displaying pixels based on the height of obstacles.
- Use 'open' function to read the input file that contains the grid map data.

#### Grayscale Representation 
- **0** (empty) should be represented as **RGB(255, 255, 255)**.
- Heights up to **9** should become progressively darker, ending at **RGB(0, 0, 0)**.

#### Character Colors 
- The car (C) is displayed in **orange (RGB(255, 165, 0))**.
- The human (H) is shown in **brown (RGB(65, 41, 35))**

#### Configuration of plot
- Set up the plot so that:
    - The **y+ axis** points upward.
    - The **x+ axis** points to the right.
- This orientation should be represented in the plot layout, not specific to any object’s orientation (such as the car).
- <!> Use `origin='lower'` in `imshow` for `matplotlib` to ensure the **y-axis points upward** in the plot display.
- <!> The axis of grid map is true.

#### Object Direction
- <!> The **car** is set to move along the **positive x-axis direction** like below.
```
  y
  ▲
  │
  │
 Car ────▶ x
```

#### Immediate Visualization
- <!> Display the visualization immediately.

Let's verify step by step.

