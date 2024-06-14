import React, { useEffect } from 'react';
import {GeoJSON, MapContainer, TileLayer, GeoJSONProps} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L, {PathOptions} from 'leaflet';
import {GeoJsonObject} from "geojson";
import * as geojson from "geojson";

interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: GeoJsonObject | null;
}

const MapViewer: React.FC<MapViewerProps> = ({ center, zoom , geoJson}) => {

    //const geoJsonData = JSON.parse(geoJson);

    useEffect(() => {
        // Убеждаемся, что Leaflet использует правильные иконки
        delete (L as any).Icon.Default.prototype._getIconUrl;

        L.Icon.Default.mergeOptions({
            iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png').default,
            iconUrl: require('leaflet/dist/images/marker-icon.png').default,
            shadowUrl: require('leaflet/dist/images/marker-shadow.png').default,
        });
    }, []);

    const geoJsonStyle: GeoJSONProps['style'] = (feature) => {
        return {
            color: "#ee5858", // Цвет линии (stroke)
            weight: 2, // Толщина линии
            fillColor: "#a11ae0", // Цвет заполнения (fill)
            fillOpacity: 0.5, // Прозрачность заполнения
        };
    };

    const colorize = (feature?: geojson.Feature<geojson.GeometryObject, any> | undefined): PathOptions => {
        return {
            color: feature!.properties.fill,
        };
    }

    return (
        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }}>
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            />
            {geoJson && <GeoJSON data={geoJson} style={colorize} />}
        </MapContainer>
    );
}

export default MapViewer;
