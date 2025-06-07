mapboxgl.accessToken = 'pk.eyJ1IjoiamFheWFudGgiLCJhIjoiY2xpcHVpMHlzMG01MDNmbGI1NTZoNDVpciJ9.GO9DSZT5E9tGdEKg8WAp9Q';

// Initialize the map
var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v11',
    center: [80.23021913617647, 12.993036661158213], // Default coordinates
    zoom: 17
});

// Array to store all markers
var markers = [];

// GeoJSON object for path
var path = {
    'type': 'FeatureCollection',
    'features': [{
        'type': 'Feature',
        'geometry': {
            'type': 'LineString',
            'coordinates': []
        }
    }]
};

// Fetch GPS coordinates from Flask
async function fetchCoordinates() {
    try {
        const response = await fetch('http://localhost:5000/get_coordinates');
        const data = await response.json();
        console.log("Received GPS Data:", data);

        if (data.length === 2) { // Ensure correct data format
            let lat = data[0];
            let lng = data[1];

            updateMarkers([lng, lat]);
        }
    } catch (error) {
        console.error("Error fetching coordinates:", error);
    }
}

// Function to create a custom marker
function createMarkerElement(color) {
    var markerElement = document.createElement('div');
    markerElement.style.width = '12px';
    markerElement.style.height = '12px';
    markerElement.style.backgroundColor = color;
    markerElement.style.borderRadius = '50%'; // Circular shape
    markerElement.style.border = '2px solid white';
    return markerElement;
}

// Update marker positions and path
function updateMarkers(newCoord) {
    // Change all previous markers to green
    markers.forEach(markerObj => {
        markerObj.element.style.backgroundColor = 'green';
    });

    // Create a new marker for the latest position (blue)
    var newMarkerElement = createMarkerElement('blue');
    var newMarker = new mapboxgl.Marker({
        element: newMarkerElement
    })
    .setLngLat(newCoord)
    .addTo(map);

    // Store the marker reference
    markers.push({ marker: newMarker, element: newMarkerElement });

    // Center the map to the latest position
    map.setCenter(newCoord);

    // Add new coordinate to the path
    path.features[0].geometry.coordinates.push(newCoord);

    // Update the path if the source exists
    if (map.getSource('path')) {
        map.getSource('path').setData(path);
    } else {
        // If source does not exist, add it
        map.addSource('path', {
            type: 'geojson',
            data: path
        });

        map.addLayer({
            id: 'path',
            type: 'line',
            source: 'path',
            paint: {
                'line-color': '#4287f5',
                'line-opacity': 0.8,
                'line-width': 3
            }
        });
    }
}

// Fetch and update marker every second
setInterval(fetchCoordinates, 1000);
