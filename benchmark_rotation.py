import time
import board
import displayio
import random

display = board.DISPLAY

# Benchmark Configuration
NUM_FRAMES = 100
print(f"Display Size: {display.width}x{display.height}")
print(f"Display Rotation: {display.rotation}")

# 1. Full Screen Fill Benchmark
print("\n--- Benchmark 1: Full Screen Fill (Solid Color) ---")
bitmap = displayio.Bitmap(display.width, display.height, 1)
palette = displayio.Palette(1)
palette[0] = 0x00FF00 # Green

tile_grid = displayio.TileGrid(bitmap, pixel_shader=palette)
group = displayio.Group()
group.append(tile_grid)
display.root_group = group

display.refresh()

start_time = time.monotonic()
for i in range(NUM_FRAMES):
    # Toggle color to force redraw
    palette[0] = 0xFF0000 if i % 2 == 0 else 0x00FF00
    display.refresh()
end_time = time.monotonic()

duration = end_time - start_time
fps = NUM_FRAMES / duration
print(f"Time: {duration:.3f}s, FPS: {fps:.2f}")


# 2. Large Bitmap Blit (Pattern)
print("\n--- Benchmark 2: Large Bitmap Blit (Checkerboard) ---")

src_bitmap = displayio.Bitmap(display.width, display.height, 2)
src_palette = displayio.Palette(2)
src_palette[0] = 0x000000
src_palette[1] = 0xFFFFFF

# Create a simple pattern
print("Generating pattern...")
for y in range(0, display.height, 10):
    for x in range(0, display.width, 10):
        if (x + y) % 20 == 0:
            # Draw a 10x10 block roughly
            for dy in range(10):
                if y+dy < display.height:
                    for dx in range(10):
                         if x+dx < display.width:
                             src_bitmap[x+dx, y+dy] = 1

tile_grid = displayio.TileGrid(src_bitmap, pixel_shader=src_palette)
group = displayio.Group()
group.append(tile_grid)
display.root_group = group
display.refresh()

print("Starting benchmark...")
start_time = time.monotonic()
for i in range(NUM_FRAMES):
    # Jitter the position to force full redraw logic
    tile_grid.x = 1 if i % 2 == 0 else 0
    display.refresh()
end_time = time.monotonic()

duration = end_time - start_time
fps = NUM_FRAMES / duration
print(f"Time: {duration:.3f}s, FPS: {fps:.2f}")
