// Joint-history heatmap painter: Blazor computes one RGBA pixel per (sample, joint) cell and hands the buffer over;
// the canvas is that many pixels and CSS scales it up (image-rendering: pixelated).
export function paint(canvas, rgba, width, height) {
  if (!canvas) return;
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.putImageData(new ImageData(new Uint8ClampedArray(rgba), width, height), 0, 0);
}
