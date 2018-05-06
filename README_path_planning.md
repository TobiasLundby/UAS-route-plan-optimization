# Path planner instructions
The folder 'results' needs to be created manually in the git root folder
## Packages needed:
 - math: _pow_, _sqrt_, _pi_, and _cos_; basic math functions
 - termcolor: _colored_; terminal output colors
 - pyproj: _Geod_; Great-circle distance calculator
 - datetime: date formatting
 - time: current EPOCH time
 - pytz: local time conversion
 - csv: save as CSV file
 - heapq: _*_; Priority heap (check later)
 - libs.coordinate: _coordinate_transform_; custom coordinate transform class
 - libs.map_plotter: _map_plotter_; custom map plotter class
 - libs.memory_usage: _memory_usage_; custom memory usage class
 - data_sources.no_fly_zones.kml_reader: _kml_no_fly_zones_parser_; custom KML polygon reader class
 - shapely: _geometry_; used for calculating distances to polygons / no-fly zones
 - scikit-image: _skimage.measure_ -> _approximate_polygon_; Ramer-Douglas-Peucker algorithm
    - http://scikit-image.org/docs/stable/api/skimage.measure.html#approximate-polygon
    - Can handle 2D data
    - Replaced with the package below since it can handle MD data and not just 2D
 - rdp: _rdp_; Ramer-Douglas-Peucker algorithm
    - http://rdp.readthedocs.io/
    - Can handle MD data
