#!/usr/bin/env python3
"""
Test script to preview the forest green color scheme for the firefighting robot GUI.
This creates a simple preview window showing the color palette and some UI elements.
"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

def create_color_preview():
    """Create a preview window showing the forest green color scheme"""
    root = tk.Tk()
    root.title("Forest Green Color Scheme Preview - Firefighting Robot GUI")
    root.geometry("800x600")
    root.configure(bg="#f0f8f0")
    
    # Forest green color palette
    colors = {
        'Primary Forest Green': '#1a3d1a',
        'Secondary Forest Green': '#2d5a2d', 
        'Light Forest Green': '#4a7c4a',
        'Background': '#f0f8f0',
        'Panel Background': '#e8f5e8',
        'Text': '#1a3d1a',
        'Fire Red': '#ff4444'
    }
    
    # Create main frame
    main_frame = ttk.Frame(root, padding=20)
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Title
    title_label = tk.Label(main_frame, text="🌲 Forest Fire Detection System - Color Preview", 
                          font=("Segoe UI", 16, "bold"), bg="#f0f8f0", fg="#1a3d1a")
    title_label.pack(pady=(0, 20))
    
    # Color palette display
    color_frame = ttk.LabelFrame(main_frame, text="Color Palette", padding=10)
    color_frame.pack(fill=tk.X, pady=(0, 20))
    
    row = 0
    for name, color in colors.items():
        color_label = tk.Label(color_frame, text=name, font=("Segoe UI", 10), 
                              bg="#e8f5e8", fg="#1a3d1a")
        color_label.grid(row=row, column=0, sticky="w", padx=10, pady=5)
        
        color_swatch = tk.Frame(color_frame, width=100, height=30, bg=color, 
                               relief="solid", borderwidth=1)
        color_swatch.grid(row=row, column=1, padx=10, pady=5)
        
        hex_label = tk.Label(color_frame, text=color, font=("Consolas", 9), 
                            bg="#e8f5e8", fg="#1a3d1a")
        hex_label.grid(row=row, column=2, sticky="w", padx=10, pady=5)
        
        row += 1
    
    # UI Elements Preview
    ui_frame = ttk.LabelFrame(main_frame, text="UI Elements Preview", padding=10)
    ui_frame.pack(fill=tk.BOTH, expand=True)
    
    # Buttons
    button_frame = ttk.Frame(ui_frame)
    button_frame.pack(fill=tk.X, pady=(0, 10))
    
    tk.Button(button_frame, text="Emergency Stop", bg="#ff4444", fg="white",
              font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4).pack(side=tk.LEFT, padx=5)
    
    tk.Button(button_frame, text="Switch to Manual", bg="#2d5a2d", fg="white",
              font=("Segoe UI", 10, "bold"), relief="flat", padx=8, pady=4).pack(side=tk.LEFT, padx=5)
    
    tk.Button(button_frame, text="Clear Paths", bg="#4a7c4a", fg="white",
              font=("Segoe UI", 10), relief="flat", padx=8, pady=4).pack(side=tk.LEFT, padx=5)
    
    # Status indicators
    status_frame = ttk.Frame(ui_frame)
    status_frame.pack(fill=tk.X, pady=(0, 10))
    
    tk.Label(status_frame, text="System Status:", font=("Segoe UI", 10, "bold"), 
             bg="#e8f5e8", fg="#1a3d1a").pack(side=tk.LEFT)
    
    tk.Label(status_frame, text="● Online", font=("Segoe UI", 10), 
             bg="#e8f5e8", fg="#2d5a2d").pack(side=tk.LEFT, padx=(10, 0))
    
    tk.Label(status_frame, text="● Fire Detected", font=("Segoe UI", 10), 
             bg="#e8f5e8", fg="#ff4444").pack(side=tk.LEFT, padx=(10, 0))
    
    # Map preview
    map_frame = ttk.LabelFrame(ui_frame, text="Map Preview", padding=10)
    map_frame.pack(fill=tk.BOTH, expand=True)
    
    # Create matplotlib figure for map preview
    fig, ax = plt.subplots(figsize=(6, 4))
    fig.patch.set_facecolor("#f0f8f0")
    ax.set_facecolor("#e8f5e8")
    ax.grid(True, color="#2d5a2d", alpha=0.3)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_title("Firefighting Robot Map", color="#1a3d1a", fontsize=12)
    
    # Add sample robot positions and paths
    ax.plot([-5, -3, -1, 1, 3], [2, 1, 0, -1, -2], '-', color='#2d5a2d', linewidth=2, alpha=0.8, label='Husky Path')
    ax.plot([-4, -2, 0, 2, 4], [3, 2, 1, 0, -1], '-', color='#4a7c4a', linewidth=2, alpha=0.8, label='Drone Path')
    
    # Robot positions
    ax.plot(-1, 0, 's', markersize=12, color='#2d5a2d', markeredgecolor='#1a3d1a', markeredgewidth=2, label='Husky')
    ax.plot(2, 1, '^', markersize=12, color='#4a7c4a', markeredgecolor='#2d5a2d', markeredgewidth=2, label='Drone')
    
    # Fire positions
    ax.scatter([3, -2, 0], [2, -3, 4], c='#ff4444', s=100, edgecolors='#cc0000', linewidth=2, label='Fire')
    
    ax.legend(loc='upper right', fontsize=8)
    
    # Embed matplotlib in tkinter
    canvas = FigureCanvasTkAgg(fig, master=map_frame)
    canvas.draw()
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    # Description
    desc_frame = ttk.Frame(main_frame)
    desc_frame.pack(fill=tk.X, pady=(10, 0))
    
    description = """
Forest Green Color Scheme Features:
• Primary colors: Deep forest greens (#1a3d1a, #2d5a2d, #4a7c4a)
• Background: Light forest green (#f0f8f0) for better readability
• Fire elements: Bright red (#ff4444) for high visibility and urgency
• Professional appearance suitable for firefighting operations
• High contrast for outdoor/emergency use
• Consistent with forest/nature theme for firefighting robots
    """
    
    tk.Label(desc_frame, text=description, font=("Segoe UI", 9), 
             bg="#f0f8f0", fg="#1a3d1a", justify=tk.LEFT).pack(anchor="w")
    
    return root

if __name__ == "__main__":
    root = create_color_preview()
    root.mainloop()