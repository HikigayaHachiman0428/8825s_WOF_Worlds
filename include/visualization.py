import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import Canvas
import re
from matplotlib.widgets import Button
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler


def read_file(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    return content


def write_file(file_path, content):
    with open(file_path, 'w') as file:
        file.write(content)


class BezierCurveEditor:
    def __init__(self, file_path):
        plt.close('all')  # Close all existing figures
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(left=0.12, bottom=0.065, top=0.94, right=0.9)
        # self.ax.invert_yaxis()
        self.ax.set_title('sto Xavier orz')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.grid(True)
        # blue gate
        self.ax.plot([2*30.48, -2*30.48], [-4*30.48, -4*30.48], c='b')
        self.ax.plot([-2*30.48, -2*30.48], [-6*30.48, -4*30.48], c='b')
        self.ax.plot([2*30.48, 2*30.48], [-6*30.48, -4*30.48],  c='b')
        # red gate
        self.ax.plot([2*30.48, -2*30.48], [4*30.48, 4*30.48], c='r')
        self.ax.plot([-2*30.48, -2*30.48], [6*30.48, 4*30.48], c='r')
        self.ax.plot([2*30.48, 2*30.48], [6*30.48, 4*30.48], c='r')
        # blue bar
        self.ax.plot([4*30.48, 6*30.48],
                     [6*30.48, 4*30.48], c='b', linewidth=6.0325)
        self.ax.plot([-4*30.48, -6*30.48],
                     [6*30.48, 4*30.48], c='b', linewidth=6.0325)
        # red bar
        self.ax.plot([4*30.48, 6*30.48],
                     [-6*30.48, -4*30.48], c='r', linewidth=6.0325)
        self.ax.plot([-4*30.48, -6*30.48],
                     [-6*30.48, -4*30.48], c='r', linewidth=6.0325)
        # kan
        self.ax.plot([4*30.48, 4*30.48], [-2*30.48, 2*30.48],
                     color='black', linewidth=6.0325)
        self.ax.plot([4*30.48, -4*30.48], [0*30.48, 0*30.48],
                     color='black', linewidth=6.0325)
        self.ax.plot([-4*30.48, -4*30.48], [-2*30.48, 2*30.48],
                     color='black', linewidth=6.0325)
        self.ax.set_xticks(
            [i * 30.48 for i in range(-6, 6, 2)])
        self.ax.set_yticks(
            [i * 30.48 for i in range(-6, 6, 2)])
        self.ax.grid(which='both', linestyle='--', linewidth=0.5)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-365.76/2, 365.76/2)
        self.ax.set_ylim(-365.76/2, 365.76/2)

        # Create a revert button
        self.revert_button_ax = plt.axes([0.8, 0.1, 0.1, 0.04])
        self.revert_button = Button(self.revert_button_ax, 'Revert')
        self.revert_button.on_clicked(self.revert_button_click)

        # Create a save button
        self.save_button_ax = plt.axes([0.8, 0.025, 0.1, 0.04])
        self.save_button = Button(self.save_button_ax, 'Save')
        self.save_button.on_clicked(self.save_button_click)

        self.curves = []
        self.control_points = []
        self.originalLine = []

        self.read_control_points_from_h(file_path)

        self.selected_point_index = None
        self.selected_curve_index = None
        self.preselected_point_index = None
        self.preselected_curve_index = None
        self.line_annotation = None

        self.draw_control_points()
        self.draw_curves()
        # Store a copy of the original control points
        self.original_curves = [curve.copy() for curve in self.curves]

        self.cid_press = self.fig.canvas.mpl_connect(
            'button_press_event', self.on_click)
        self.cid_motion = self.fig.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)
        self.cid_release = self.fig.canvas.mpl_connect(
            'button_release_event', self.release_point)
        # Arrow properties
        self.arrow_length = 20
        self.arrow_width = 10
        self.skipFirstStep = True
        # Draw arrows for each curve
        self.draw_arrows()
        # Create a refresh button
        self.refresh_button_ax = plt.axes(
            [0.1, 0.065, 0.1, 0.04])  # Adjust the position
        self.refresh_button = Button(self.refresh_button_ax, 'Refresh')
        self.refresh_button.on_clicked(self.refresh_button_click)

    def read_control_points_from_h(self, file_path):
        # Define the pattern to match the start of waypoint definition
        start_pattern = r"wayPointMat\s*\{"
        # Define the pattern to match coordinates
        coordinate_pattern = r"\{\s*(-?\d+\.?\d*),\s*(-?\d+\.?\d*)\s*\}"
        waypoints = []
        is_reading_waypoints = False
        current_waypoint = []
        startStep = True
        with open(file_path) as file:
            for line in file:
                if not line.strip().startswith("//"):  # Exclude commented lines
                    if is_reading_waypoints:
                        match = re.search(coordinate_pattern, line)
                        if match:
                            # print(match)
                            x, y = map(float, match.groups())
                            current_waypoint.append((x, y))
                        if len(current_waypoint) == 4:
                            waypoints.append(current_waypoint)
                            current_waypoint = []
                            startStep = False
                            is_reading_waypoints = False
                    else:
                        if re.search(start_pattern, line):
                            # print(line)
                            is_reading_waypoints = True
                            if startStep:
                                current_waypoint.append((-142, -123))

        # Append control points for each curve
        for points in waypoints:
            curve_points = []
            for point in points:
                curve_points.append(point)
            self.curves.append(curve_points)
        # print(self.curves)

    def draw_control_points(self):
        if hasattr(self, 'control_point_plots'):
            for plot in self.control_point_plots:
                plot.remove()

        self.control_point_plots = []
        self.point_annotations = []

        curveIndex = 0
        for points in self.curves:
            # Only draw starting and ending points if curve is unselected
            if curveIndex != self.preselected_curve_index:
                start_point = points[0]
                end_point = points[-1]
                x_start, y_start = start_point
                x_end, y_end = end_point
                # Draw starting point
                control_point_start, = self.ax.plot(x_start, y_start, 'ro')
                self.control_point_plots.append(control_point_start)
                # Draw ending point
                control_point_end, = self.ax.plot(x_end, y_end, 'ro')
                self.control_point_plots.append(control_point_end)
            else:  # If curve is selected, draw all control points
                for point in points:
                    x, y = point
                    control_point, = self.ax.plot(x, y, 'go')
                    self.control_point_plots.append(control_point)
            curveIndex += 1

    def draw_curves(self):
        if hasattr(self, 'curve_plots'):
            for curve_plot in self.curve_plots:
                curve_plot.remove()

        self.curve_plots = []
        for curve in self.curves:
            points = np.array(curve)
            t = np.linspace(0, 1, 100)
            bezier_curve = np.array(
                [self.de_casteljau(points, t_val) for t_val in t])
            curve_plot, = self.ax.plot(
                bezier_curve[:, 0], bezier_curve[:, 1], 'b-')
            self.curve_plots.append(curve_plot)

    def draw_arrows(self):
        if hasattr(self, 'arrow_annotations'):
            for annotation in self.arrow_annotations:
                annotation.remove()

        self.arrow_annotations = []

        for curve in self.curves:
            if len(curve) >= 2:
                start_point = curve[-2]
                end_point = curve[-1]
                dx = end_point[0] - start_point[0]
                dy = end_point[1] - start_point[1]
                angle = np.arctan2(dy, dx)

                arrow_x = end_point[0] - self.arrow_length * np.cos(angle)
                arrow_y = end_point[1] - self.arrow_length * np.sin(angle)

                arrow_annotation = self.ax.annotate('', xy=end_point, xytext=(arrow_x, arrow_y),
                                                    arrowprops=dict(
                                                        arrowstyle='->', lw=1.5, color='black'),
                                                    annotation_clip=False)
                self.arrow_annotations.append(arrow_annotation)

    def de_casteljau(self, points, t):
        if len(points) == 1:
            return points[0]

        new_points = []
        for i in range(len(points) - 1):
            x = (1 - t) * points[i][0] + t * points[i + 1][0]
            y = (1 - t) * points[i][1] + t * points[i + 1][1]
            new_points.append((x, y))

        return self.de_casteljau(new_points, t)

    def on_click(self, event):
        if event.inaxes == self.ax:
            for i, points in enumerate(self.curves):
                for j, point in enumerate(points):
                    x, y = point  # Access x and y directly from the tuple
                    if abs(event.xdata - x) <= 5 and abs(event.ydata - y) <= 5:
                        self.selected_curve_index = i
                        self.selected_point_index = j
                        self.preselected_curve_index = i
                        self.preselected_point_index = j
                        self.original_curves = [curve.copy()
                                                for curve in self.curves]
                        self.draw_control_points()
                        self.draw_curves()
                        self.draw_arrows()
                        plt.pause(0.01)
                        break

    def on_motion(self, event):
        if event.inaxes == self.ax and self.selected_point_index is not None:
            x, y = event.xdata, event.ydata
            self.curves[self.selected_curve_index][self.selected_point_index] = (
                x, y)
            print(self.curves[self.selected_curve_index])
            self.update_annotation(x, y)  # Update annotation
            self.draw_curves()
            self.draw_control_points()
            self.draw_arrows()
            plt.pause(0.01)

    def release_point(self, event):
        if event.inaxes == self.ax:
            self.selected_point_index = None
            self.selected_curve_index = None

            self.draw_curves()
            self.draw_control_points()
            self.draw_arrows()
            plt.pause(0.01)

    def update_annotation(self, x, y):
        if hasattr(self, 'point_annotation'):
            self.point_annotation.remove()
        self.point_annotation = self.ax.annotate(
            f'({x:.2f}, {y:.2f})', (x, y), textcoords="offset points", xytext=(5, 5), ha='center')
        plt.draw()

    def save_button_click(self, event):
        file_path = "include/autonomous.h"  # Update with the actual file path
        # print(self.curves)
        if hasattr(self, 'point_annotation'):
            self.point_annotation.remove()
        self.modify_bezier_curve(file_path, self.curves)
        print("Control points saved to file.")

    def revert_button_click(self, event):
        # Revert changes by restoring the original control points
        self.curves = [curve.copy() for curve in self.original_curves]
        self.draw_curves()
        self.draw_control_points()
        # if hasattr(self, 'point_annotation'):
        #     self.point_annotation.remove()
        plt.draw()

    def modify_bezier_curve(self, file_path, new_control_points_lists):
        # Define the patterns
        start_pattern = re.compile(r'wayPointMat{')
        comment_pattern = re.compile(r'^\s*//')
        print(new_control_points_lists)
        waypoints = []
        with open(file_path, 'r') as file:
            lines = file.readlines()

        # Find lines containing 'wayPointMat{' and not starting with '//'
        for i, line in enumerate(lines):
            if start_pattern.search(line) and not comment_pattern.match(line):
                waypoints.append(i)

        # Modify the following four lines for each waypoint, skipping the first line
        cnt = 0
        for waypoint in waypoints:
            for j in range(1, 5):  # Modify the following four lines
                if cnt == 0 and j == 1:  # Skip modifying the first line
                    continue
                # print(waypoint, waypoint + j)
                new_x, new_y = new_control_points_lists[cnt][j-1]

                # Modify the line
                lines[waypoint +
                      j] = f'\t\t{{%.2f, %.2f}}' % (new_x, new_y)

                if j == 4:
                    lines[waypoint + j] += '};\n'
                else:
                    lines[waypoint + j] += ',\n'
            cnt += 1

        # Write the modified content back to the file
        with open(file_path, 'w') as file:
            file.writelines(lines)

    def refresh_button_click(self, event):
        # Re-read the control points from the original file
        self.curves = []
        self.control_points = []
        self.originalLine = []
        self.read_control_points_from_h("include/autonomous.h")
        print(self.curves)
        self.selected_point_index = None
        self.selected_curve_index = None
        self.preselected_point_index = None
        self.preselected_curve_index = None
        self.line_annotation = None
        # Redraw the curves and control points
        self.draw_curves()
        self.draw_control_points()
        self.draw_arrows()
        # if hasattr(self, 'point_annotation'):
        #     self.point_annotation.remove()
        plt.draw()
        plt.pause(0.01)


def main():
    cwd = os.getcwd()  # Get the current working directory (cwd)
    files = os.listdir(cwd)  # Get all the files in that directory
    print("Files in %r: %s" % (cwd, files))
    editor = BezierCurveEditor('include/autonomous.h')
    plt.show()


if __name__ == "__main__":
    main()
