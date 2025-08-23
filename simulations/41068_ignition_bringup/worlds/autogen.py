#!/usr/bin/env python3
import random
import math

def generate_random_position(min_radius=3, max_radius=25, avoid_center=True):
    """Generate random position avoiding center area if needed"""
    while True:
        x = random.uniform(-max_radius, max_radius)
        y = random.uniform(-max_radius, max_radius)
        
        # Avoid center area if specified
        if avoid_center and (abs(x) < min_radius and abs(y) < min_radius):
            continue
            
        return x, y

def generate_tree(tree_type, name, x, y, scale_variation=0.2, rotation_random=True):
    """Generate a tree with random scaling and rotation"""
    
    # Random scaling
    scale = 1.0 + random.uniform(-scale_variation, scale_variation)
    
    # Random rotation
    rotation = random.uniform(0, 2 * math.pi) if rotation_random else 0
    
    # Tree URIs
    tree_uris = {
        'oak': 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak%20tree',
        'pine': 'model://pine_tree'
    }
    
    return f"""    <include>
      <uri>{tree_uris[tree_type]}</uri>
      <name>{name}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0.05 {rotation:.2f}</pose>
      
      <static>true</static>
    </include>"""

def generate_rock(rock_type, name, x, y, scale_variation=0.3):
    """Generate a rock with random scaling and rotation"""
    
    scale = 1.0 + random.uniform(-scale_variation, scale_variation)
    rotation = random.uniform(0, 2 * math.pi)
    
    rock_uris = {
        1: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%201',
        2: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%202',
        3: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%203',
        4: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%204',
        5: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%205',
        6: 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling%20Rock%206'
    }
    
    return f"""    <include>
      <uri>{rock_uris[rock_type]}</uri>
      <name>{name}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0.35 {rotation:.2f}</pose> #THE ROCK IS RAISED UP HERE COS ITS SMALL
      
      <static>true</static>
    </include>"""

def generate_pine_cluster(cluster_id, center_x, center_y, num_trees=None):
    """Generate a cluster of pine trees"""
    if num_trees is None:
        num_trees = random.randint(3, 7)
    
    trees = []
    for i in range(num_trees):
        # Offset from cluster center (closer together than random placement)
        offset_x = random.uniform(-12, 12)
        offset_y = random.uniform(-12, 12)
        
        trees.append(generate_tree(
            'pine', 
            f'pine_cluster_{cluster_id}_tree_{i+1}', 
            center_x + offset_x, 
            center_y + offset_y, 
            scale_variation=0.3
        ))
    
    return trees
    """Generate a simple geometric tree with realistic colors"""
    
    # Default values with variation
    trunk_radius = trunk_radius or random.uniform(0.2, 0.4)
    trunk_height = trunk_height or random.uniform(3, 6)
    foliage_radius = foliage_radius or random.uniform(1.5, 3.0)
    
    # Realistic brown trunk colors
    trunk_r = random.uniform(0.35, 0.55)
    trunk_g = random.uniform(0.2, 0.35)
    trunk_b = random.uniform(0.1, 0.25)
    trunk_color = f"{trunk_r:.2f} {trunk_g:.2f} {trunk_b:.2f}"
    
    # Realistic green foliage colors
    foliage_r = random.uniform(0.15, 0.35)
    foliage_g = random.uniform(0.5, 0.75)
    foliage_b = random.uniform(0.15, 0.35)
    foliage_color = f"{foliage_r:.2f} {foliage_g:.2f} {foliage_b:.2f}"
    
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
      <link name="trunk">
        <collision name="trunk_collision">
          <geometry>
            <cylinder>
              <radius>{trunk_radius:.2f}</radius>
              <length>{trunk_height:.2f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="trunk_visual">
          <pose>0 0 {trunk_height/2:.2f} 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>{trunk_radius:.2f}</radius>
              <length>{trunk_height:.2f}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{trunk_color} 1</ambient>
            <diffuse>{trunk_color} 1</diffuse>
          </material>
        </visual>
      </link>
      <link name="foliage">
        <pose>0 0 {trunk_height + foliage_radius * 0.7:.2f} 0 0 0</pose>
        <collision name="foliage_collision">
          <geometry>
            <sphere>
              <radius>{foliage_radius:.2f}</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="foliage_visual">
          <geometry>
            <sphere>
              <radius>{foliage_radius:.2f}</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>{foliage_color} 1</ambient>
            <diffuse>{foliage_color} 1</diffuse>
          </material>
        </visual>
      </link>
      <joint name="trunk_foliage" type="fixed">
        <child>foliage</child>
        <parent>trunk</parent>
      </joint>
    </model>"""

def generate_forest_world():
    """Generate a complete forest world"""
    
    world_content = []
    
    # Generate Oak trees (35-50 trees) with 20% variation
    num_oaks = random.randint(30, 40)
    for i in range(num_oaks):
        x, y = generate_random_position(min_radius=3, max_radius=23)
        world_content.append(generate_tree('oak', f'oak_{i+1}', x, y, scale_variation=0.2))
    
    # Generate Pine tree clusters (4-6 clusters)
    num_pine_clusters = random.randint(8, 10)
    for cluster in range(num_pine_clusters):
        center_x, center_y = generate_random_position(min_radius=5, max_radius=23)
        cluster_trees = generate_pine_cluster(cluster + 1, center_x, center_y)
        world_content.extend(cluster_trees)
    
    # Generate individual Pine trees (15-25 trees) with 30% variation
    num_individual_pines = random.randint(20, 30)
    for i in range(num_individual_pines):
        x, y = generate_random_position(min_radius=4, max_radius=23)
        world_content.append(generate_tree('pine', f'pine_individual_{i+1}', x, y, scale_variation=0.3))
    
    # Generate rocks in clusters using the provided rock models
    num_rock_clusters = random.randint(20, 25)
    for cluster in range(num_rock_clusters):
        # Pick a center for the cluster
        center_x, center_y = generate_random_position(min_radius=3, max_radius=23)
        
        # Generate 2-5 rocks around this center
        rocks_in_cluster = random.randint(2, 5)
        for rock in range(rocks_in_cluster):
            # Offset from cluster center
            offset_x = random.uniform(-8, 8)
            offset_y = random.uniform(-8, 8)
            rock_type = random.randint(1, 1) #CHANGED TO 1 SO ONLY BIG ROCK SELECTED
            
            world_content.append(generate_rock(
                rock_type, 
                f'rock_cluster_{cluster}_rock_{rock}', 
                center_x + offset_x, 
                center_y + offset_y,
                scale_variation=0.4
            ))
    
    return '\n'.join(world_content)

# Generate the forest content
if __name__ == "__main__":
    # Generate the content
    forest_content = generate_forest_world()
    
    # Write to file
    output_filename = "generated_forest.sdf"
    
    with open(output_filename, 'w') as f:
        f.write("<!-- Generated Forest Content -->\n")
        f.write("<!-- Copy this content into your world file after the ground plane -->\n\n")
        f.write(forest_content)
        f.write("\n\n<!-- End Generated Content -->")
    
    print(f"Forest content generated and saved to: {output_filename}")
    print(f"Generated content includes:")
    
    # Count the objects for summary
    lines = forest_content.split('\n')
    oak_count = len([line for line in lines if 'oak_' in line and '<name>' in line])
    pine_count = len([line for line in lines if 'pine_' in line and '<name>' in line])
    rock_count = len([line for line in lines if 'rock_cluster_' in line and '<name>' in line])
    
    print(f"- {oak_count} Oak trees")
    print(f"- {pine_count} Pine trees") 
    print(f"- {rock_count} Rocks")
    print(f"- Total objects: {oak_count + pine_count + rock_count}")
    print(f"\nTo use: Copy the contents of {output_filename} into your world file.")