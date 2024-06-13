import React, { useEffect } from 'react';
import {GeoJSON, MapContainer, TileLayer} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: string;
}

const MapViewer: React.FC<MapViewerProps> = ({ center, zoom , geoJson}) => {

    const geoJsonData = JSON.parse(geoJson);

    useEffect(() => {
        // Убеждаемся, что Leaflet использует правильные иконки
        delete (L as any).Icon.Default.prototype._getIconUrl;

        L.Icon.Default.mergeOptions({
            iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png').default,
            iconUrl: require('leaflet/dist/images/marker-icon.png').default,
            shadowUrl: require('leaflet/dist/images/marker-shadow.png').default,
        });
    }, []);

    return (
        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }}>
            <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            />

            <GeoJSON data={geoJsonData}/>

        </MapContainer>
    );
};

export default MapViewer;
