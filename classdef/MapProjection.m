classdef MapProjection < handle
    properties
        ref_timestamp;%{0};
	    ref_lat;%{0.0};
	    ref_lon;%{0.0};
	    ref_sin_lat;%{0.0};
	    ref_cos_lat;%{0.0};
	    ref_init_done;%{false};
        
    end

    methods
        function obj = MapProjection()

%             initReference(obj,lat_0,lon_0,timestamp);
            obj.ref_init_done = false;
        end

        function MapProjection_init(lat_0,lon_0,timestamp)

            initReference(obj,lat_0,lon_0,timestamp);
            
        end

        function obj = initReference(obj,lat_0,lon_0,timestamp)
        
            obj.ref_timestamp = timestamp;
	        obj.ref_lat = radians(lat_0);
	        obj.ref_lon = radians(lon_0);
	        obj.ref_sin_lat = sin(obj.ref_lat);
	        obj.ref_cos_lat = cos(obj.ref_lat);
	        obj.ref_init_done = true;

        end

        function ret = isInitialized(obj)
            ret = obj.ref_init_done;
        end

        function res = project(obj,lat,lon)

            global CONSTANTS_RADIUS_OF_EARTH;

	        lat_rad = radians(lat);
	        lon_rad = radians(lon);
        
	        sin_lat = sin(lat_rad);
	        cos_lat = cos(lat_rad);
        
	        cos_d_lon = cos(lon_rad - obj.ref_lon);
        
	        arg = saturation(obj.ref_sin_lat * sin_lat + obj.ref_cos_lat * cos_lat * cos_d_lon, -1.0,  1.0);
	        c = acos(arg);
        
	        k = 1.0;
        
	        if fabs(c) > 0 
		        k = (c / sin(c));
            end
        
	        x = (k * (obj.ref_cos_lat * sin_lat - obj.ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	        y = (k * cos_lat * sin(lon_rad - obj.ref_lon) * CONSTANTS_RADIUS_OF_EARTH);
        
            res = [x y]';
        end

        function [lat,lon]=reproject(obj,x,y)
            
            global CONSTANTS_RADIUS_OF_EARTH;
            
            x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
	        y_rad = y / CONSTANTS_RADIUS_OF_EARTH;

	        c = sqrt(x_rad * x_rad + y_rad * y_rad);
        
	        if (fabs(c) > 0)
		        sin_c = sin(c);
		        cos_c = cos(c);
        
		        lat_rad = asin(cos_c * obj.ref_sin_lat + (x_rad * sin_c * obj.ref_cos_lat) / c);
		        lon_rad = (obj.ref_lon + atan2(y_rad * sin_c, c * obj.ref_cos_lat * cos_c - x_rad * obj.ref_sin_lat * sin_c));
        
		        lat = degrees(lat_rad);
		        lon = degrees(lon_rad);
        
            else
		        lat = degrees(obj.ref_lat);
		        lon = degrees(obj.ref_lon);
            end
            
        end

        function res = getProjectionReferenceLat(obj)
            res = obj.ref_lat;
        end

        function res = getProjectionReferenceLon(obj)
            res = obj.ref_lon;
        end
        function res = getProjectionReferenceTimestamp(obj)
            res = obj.ref_timestamp;
        end
    end
end

