from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QColor, QPolygon, QPen, QBrush, QFont
from PyQt5.QtCore import Qt, QPoint, QRectF
import math

class AttitudeIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.roll = 0.0  # Degrees
        self.pitch = 0.0 # Degrees
        self.setMinimumSize(200, 200)

    def setRoll(self, roll_deg):
        self.roll = roll_deg
        self.update()

    def setPitch(self, pitch_deg):
        self.pitch = pitch_deg
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2

        # 1. Background (Instrument Case)
        painter.fillRect(0, 0, w, h, Qt.black)
        
        # 2. Setup Coordinate System
        # We translate to center, rotate by -roll (because instrument rotates opposite to bank)
        # However, for an artificial horizon:
        # If plane banks LEFT (Roll < 0), horizon should appear tilted RIGHT relative to fixed plane reference.
        # Actually standard implementation: rotate the horizon line by -roll.
        
        painter.translate(cx, cy)
        painter.rotate(-self.roll)

        # 3. Draw Sky and Ground
        # Pitch translation: 
        # Visible range usually +/- 20 degrees or so.
        # Scale: pixels per degree.
        # Let's say instrument height covers 40 degrees FOV (approx).
        scale = h / 40.0 
        pitch_offset = self.pitch * scale

        # Sky (Blue)
        # Draw a huge rectangle shifted by pitch
        sky_rect = QRectF(-w*2, -h*4 + pitch_offset, w*4, h*4)
        painter.fillRect(sky_rect, QColor(0, 150, 255))

        # Ground (Brown/Orange)
        gnd_rect = QRectF(-w*2, pitch_offset, w*4, h*4)
        painter.fillRect(gnd_rect, QColor(139, 69, 19))

        # 4. Draw Horizon Line
        painter.setPen(QPen(Qt.white, 2))
        painter.drawLine(int(-w*2), int(pitch_offset), int(w*2), int(pitch_offset))

        # 5. Pitch Ladder
        # Draw lines at every 10 degrees
        painter.setPen(QPen(Qt.white, 1))
        painter.setFont(QFont("Arial", 8))
        
        for i in range(-90, 91, 10):
            if i == 0: continue
            
            y = pitch_offset - (i * scale)
            
            # Don't draw if too far off screen
            if y < -h or y > h: continue

            # Line width depends on major/minor (every 10 is major here)
            length = w / 4 if i % 10 == 0 else w / 6
            
            painter.drawLine(int(-length/2), int(y), int(length/2), int(y))
            
            # Text
            painter.drawText(int(length/2 + 5), int(y + 5), f"{abs(i)}")
            painter.drawText(int(-length/2 - 20), int(y + 5), f"{abs(i)}")

        # 6. Reset Transformations for fixed instrument parts
        painter.resetTransform()
        painter.translate(cx, cy)

        # 7. Draw Fixed Reference (The Plane) - Orange W or Wings
        painter.setPen(QPen(QColor(255, 140, 0), 3))
        painter.setBrush(Qt.NoBrush)
        
        # Wings
        # Left Wing
        painter.drawLine(-40, 0, -10, 0)
        painter.drawLine(-10, 0, 0, 10)
        # Right Wing
        painter.drawLine(40, 0, 10, 0)
        painter.drawLine(10, 0, 0, 10)
        
        # Center Dot
        painter.setBrush(QColor(255, 140, 0))
        painter.drawEllipse(-2, -2, 4, 4)
