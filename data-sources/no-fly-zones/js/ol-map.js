/*
2018-02-09 TL First version, moved from kml-viewer.html

Description: Map JS for kml-viewer.html

License: BSD 3-Clause
*/

// Use EPSG:3857 otherwise the view/projection is skewed, remember to convert the coords (GPS = WGS 84 = EPSG:4326, https://epsg.io/4326) we normally use with: ol.proj.transform([10.4, 55.9],  'EPSG:4326', 'EPSG:3857')
var projection = ol.proj.get('EPSG:3857');
var kml_url = 'http://localhost:8888/msc-ol/export.kml';
// drone_nofly_dk.kml
// KmlUasZones_2018-01-05-20-36.kml
// https://www.techgen.dk/msc/export.kml

// Empty raster layer
var raster = new ol.layer.Tile({
    source: new ol.source.OSM()
});

// Empty vector layer
var vector = new ol.layer.Vector({
    source: new ol.source.Vector()
});

// No-fly zone vector layer
var vector_no_fly_zones_dk = new ol.layer.Vector({
    source: new ol.source.Vector()
});

var map = new ol.Map({
    layers: [raster, vector, vector_no_fly_zones_dk],
    target: document.getElementById('map'),
    view: new ol.View({
        center: ol.proj.transform([11.134,56.2],  'EPSG:4326', 'EPSG:3857'), //10.4, 55.9
        projection: projection,
        displayProjection: ol.proj.get("EPSG:4326"), // Display coords in lat/lng
        zoom: 7,
        units: 'm'
    })
});

function iterate_features(){
    var features = [];
    vector_no_fly_zones_dk.getSource().forEachFeature(function(feature) {
        features.push(feature);
    });
    zone_names = [];
    zone_style = [];
    zone_style_cnt = [];
    zone_names_and_style = [];
    var i, ii;
    for (i = 0, ii = features.length; i < ii; i++) {
        zone_names.push(features[i].get('name'));
        zone_names_and_style.push(features[i].get('name').concat(' <i>(', features[i].get('styleUrl').split("uasZone_").pop(), ')</i>'));
        if (i==0) {
            // console.log(i,features[i].get('styleUrl').split("uasZone_").pop());
            zone_style.push(features[i].get('styleUrl').split("uasZone_").pop());
            zone_style_cnt.push(1);
        }else {
            var tmp_zone_style = features[i].get('styleUrl').split("uasZone_").pop();
            var k, kk;
            // console.log('size: ', zone_style.length, ' - ', zone_style.length);
            for (k = 0, kk = zone_style.length; k < kk; k++) {
                if (tmp_zone_style == zone_style[k]) {
                    // console.log(i,', int: ', k, ', ++(',zone_style_cnt.length,'): ',tmp_zone_style,"==",zone_style[k]);
                    zone_style_cnt[k]++;
                    break;
                } else if (k == kk-1) {
                    // console.log(i,', int: ', k, ', added', tmp_zone_style, "!=", zone_style[k]);
                    zone_style.push(tmp_zone_style);
                    zone_style_cnt.push(1);
                }
            }
        }
    }
    // Test to see if the count is the same: verified and commented out
    // var i, ii;
    // var tmp_sum = 0;
    // for (i = 0, ii = zone_style_cnt.length; i < ii; i++) {
    //     tmp_sum = tmp_sum + zone_style_cnt[i];
    // }
    // console.log(tmp_sum);
    document.getElementById('zone_types').innerHTML = '<p>'.concat(zone_style.join('<br>'),'</p>');
    document.getElementById('zone_types_cnt').innerHTML = '<p>'.concat(zone_style_cnt.join('<br>'),'</p>');
    document.getElementById('zones').innerHTML = '<p>'.concat(zone_names_and_style.join('<br>'),'</p>');
    //document.getElementById('zones').innerHTML = '<p>'.concat(zone_names.join('<br>'),'</p>');
    document.getElementById('total_map_zones').innerHTML = '<p>Total map zones: '.concat(features.length,'</p>');
    //console.log(features.length);
}

function add_no_fly_zones(){
    var source_nofly = new ol.source.Vector({
        url: kml_url,
        format: new ol.format.KML({
            extractStyles: true,
            extractAttributes: true,
            showLabels: true
        }),
    });
    // No need for clearing the layer
        // var vector_source = layerVector.getSource();
        // vector_source.clear()
    map.removeLayer(vector_no_fly_zones_dk);
    vector_no_fly_zones_dk = new ol.layer.Vector({
        source: source_nofly
    });
    map.addLayer(vector_no_fly_zones_dk);
}

// Add the zones
add_no_fly_zones();

// Inspiration: https://openlayers.org/en/latest/examples/kml.html
var displayFeatureInfo = function(pixel, text_element_id) {
    var features = [];
    map.forEachFeatureAtPixel(pixel, function(feature) {
        features.push(feature);
    });
    if (features.length > 0) {
        var info = [];
        var i, ii;
        for (i = 0, ii = features.length; i < ii; i++) {
            info.push('<u><b>Zone '.concat(i+1,'</b></u>'));
            info.push('<b style="margin-left:5px;">Name:</b> '.concat(features[i].get('name')));
            var tmp_str = features[i].get('styleUrl');
            tmp_str = tmp_str.split("uasZone_").pop();
            // console.log(tmp_str);
            info.push('<b style="margin-left:5px;">Type:</b> '.concat(tmp_str));
        }
        document.getElementById(text_element_id).innerHTML = '<p>'.concat(info.join('<br>'),'</p>') || '(unknown)';
        map.getTarget().style.cursor = 'pointer';
    } else {
        document.getElementById(text_element_id).innerHTML = '<i>none</i>';
        map.getTarget().style.cursor = '';
    }
};

// Execute when page loaded
window.onload = function(){
    // Iterate the features but wait a bit for the map to render
    setTimeout(iterate_features, 500);
};
// Interactive elements
map.on('pointermove', function(evt) {
    if (evt.dragging) {
        return;
    }
    var pixel = map.getEventPixel(evt.originalEvent);
    displayFeatureInfo(pixel, 'info');
});

map.on('click', function(evt) {
    displayFeatureInfo(evt.pixel,'info_clicked');
    //document.getElementById('info').innerHTML = '';
});
