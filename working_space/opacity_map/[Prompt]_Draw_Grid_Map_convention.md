(input 03_height_C_H.txt file)
# Issues

### Initial Condition
#### Opacity Map -> Grid Map
##### Character
- 0 : empty
- n(positive integer) : height of obstacle
    - Tall Building :  n == 9
    - Middle Building : n == 8
    - Low Building : 5 <= n < 8
    - Sidewalk : n == 1
- M : My Vehicle
- C : Other Vehicle
- T : Street Tree
- G : Guardrail
- H : Human


  
# Instructions
### Objective

#### Never miss any sentence with the `<!>` tag.
- The tag will appear at the beginning of the sentence.
- Double-check to ensure no tags are missed.
#### Reading and Displaying the Grid Map 
- Draw the map using **matplotlib**, displaying pixels based on the height of obstacles.
- Use 'open' function to read the input file that contains the grid map data.

#### Character Colors 
- The empty(0) : **white RGB(255, 255, 255)**
- Sidewalk(1) : **light silver RGB(211, 211, 211)**
- Buildings (5 to 9): Progressively darker shades, ending with **light gray RGB(192, 192, 192)** for height **9**
- My Vehicle(M): **blue RGB(0, 0, 255)**
- The other Vehicle(C) : **orange RGB(255, 165, 0)**.
- Street Tree (T): **green RGB(34, 139, 34)**
- Guardrail (G) : **dim silver RGB(192, 192, 192)**
- The human (H) : **brown RGB(65, 41, 35)**

#### Configuration of plot
- **My Vehicle(M)** is positioned at the origin **(0, 0)**.
- Set up the plot so that:
    - The **y+ axis** points upward.
    - The **x+ axis** points to the right.
- This orientation should be represented in the plot layout, not specific to any object’s orientation (such as the car).
- <!> Use `origin='lower'` in `imshow` for `matplotlib` to ensure the **y-axis points upward** in the plot display.
- <!> The axis of grid map is true.
- <!> Add the label which obejects

#### Immediate Visualization
- <!> Display the visualization immediately.

Let's verify step by step.

