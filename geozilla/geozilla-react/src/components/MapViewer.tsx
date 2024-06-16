import React, { useEffect, useRef } from 'react';
import { MapContainer, TileLayer, useMap } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import { GeoJsonObject } from 'geojson';
import * as geojson from 'geojson';
import L, { PathOptions } from 'leaflet';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";

interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: GeoJsonObject | null;
}

const MapViewer: React.FC<MapViewerProps> = ({ center, zoom, geoJson }) => {
    const grassLayerRef = useRef(new L.FeatureGroup());
    const roadLayerRef = useRef(new L.FeatureGroup());
    const sidewalkLayerRef = useRef(new L.FeatureGroup());
    const buildingLayerRef = useRef(new L.FeatureGroup());
    const defaultLayerRef = useRef(new L.FeatureGroup());
    const layerControlRef = useRef<L.Control.Layers | null>(null);
    const mapRef = useRef<L.Map | null>(null);

    useEffect(() => {
        // Ensure Leaflet uses the correct icons
        delete (L as any).Icon.Default.prototype._getIconUrl;

        L.Icon.Default.mergeOptions({
            iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png').default,
            iconUrl: require('leaflet/dist/images/marker-icon.png').default,
            shadowUrl: require('leaflet/dist/images/marker-shadow.png').default,
        });
    }, []);

    useEffect(() => {
        if (!geoJson) return;

        grassLayerRef.current.clearLayers();
        roadLayerRef.current.clearLayers();
        sidewalkLayerRef.current.clearLayers();
        buildingLayerRef.current.clearLayers();
        defaultLayerRef.current.clearLayers();

        const features = (geoJson as geojson.FeatureCollection).features;

        features.forEach((feature) => {
            const zoneType = feature.properties!.zoneType;

            const layer = L.geoJSON(feature, {
                style: colorize(zoneType)
            });

            const targetLayer = getLayer(zoneType);
            targetLayer.addLayer(layer);
        });
    }, [geoJson]);

    const getLayer = (zoneType: string) => {
        switch (zoneType) {
            case 'grass':
                return grassLayerRef.current;
            case 'road':
                return roadLayerRef.current;
            case 'sidewalk':
                return sidewalkLayerRef.current;
            case 'building':
                return buildingLayerRef.current;
            default:
                return defaultLayerRef.current;
        }
    };

    const colorize = (zoneType: string): PathOptions => {
        switch (zoneType) {
            case 'grass':
                return {
                    color: '#76c893', // Color for grass
                    weight: 2, // Line thickness
                    fillColor: '#76c893', // Fill color for grass
                    fillOpacity: 0.5, // Fill opacity
                };
            case 'road':
                return {
                    color: '#a1a1a1', // Color for road
                    weight: 2, // Line thickness
                    fillColor: '#a1a1a1', // Fill color for road
                    fillOpacity: 0.5, // Fill opacity
                };
            case 'sidewalk':
                return {
                    color: '#d1d1d1', // Color for sidewalk
                    weight: 2, // Line thickness
                    fillColor: '#d1d1d1', // Fill color for sidewalk
                    fillOpacity: 0.5, // Fill opacity
                };
            case 'building':
                return {
                    color: '#ff8c00', // Color for building
                    weight: 2, // Line thickness
                    fillColor: '#ff8c00', // Fill color for building
                    fillOpacity: 0.5, // Fill opacity
                };
            default:
                return {
                    color: '#ee5858', // Default color
                    weight: 2, // Line thickness
                    fillColor: '#a11ae0', // Default fill color
                    fillOpacity: 0.5, // Fill opacity
                };
        }
    };

    function MapSettings() {
        const map = useMap();

        useEffect(() => {
            if (!mapRef.current) {
                mapRef.current = map;

                map.pm.addControls({
                    position: 'topleft',
                    drawCircleMarker: false,
                    rotateMode: false,
                });

                if (layerControlRef.current === null) {
                    layerControlRef.current = L.control.layers({}, {
                        'Grass': grassLayerRef.current,
                        'Road': roadLayerRef.current,
                        'Sidewalk': sidewalkLayerRef.current,
                        'Building': buildingLayerRef.current,
                        'Default': defaultLayerRef.current
                    }, { collapsed: false }).addTo(map);

                    map.addLayer(grassLayerRef.current);
                    map.addLayer(roadLayerRef.current);
                    map.addLayer(sidewalkLayerRef.current);
                    map.addLayer(buildingLayerRef.current);
                    map.addLayer(defaultLayerRef.current);
                }
            }
        }, [map]);

        return null;
    }

    return (
        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }}>
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            />
            <MapEditor geoJson={geoJson} />
            <MapSettings />
        </MapContainer>
    );
}

interface MapEditorProps {
    geoJson: GeoJsonObject | null;
}

const MapEditor: React.FC<MapEditorProps> = ({ geoJson }) => {
    const map = useMap();

    useEffect(() => {
        if (!map) return;

        // Attach Leaflet-Geoman to the map
        map.pm.addControls({
            position: 'topleft',
            drawCircle: false,
            drawMarker: false,
            drawCircleMarker: false,
        });

        // Event handlers for geometry
        map.on('pm:create', (e) => {
            console.log('Created shape:', e);
        });

        map.on('pm:edit', (e) => {
            console.log('Edited shape:', e);
        });

        map.on('pm:remove', (e) => {
            console.log('Removed shape:', e);
        });
    }, [map, geoJson]);

    return null;
};

export default MapViewer;
