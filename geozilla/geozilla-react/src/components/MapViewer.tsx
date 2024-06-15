import React, { useEffect } from 'react';
import {GeoJSON, MapContainer, TileLayer, useMap} from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import {GeoJsonObject} from "geojson";
import * as geojson from "geojson";
import "leaflet/dist/leaflet.css";
import L, {PathOptions} from 'leaflet';
import "@geoman-io/leaflet-geoman-free";
import "@geoman-io/leaflet-geoman-free/dist/leaflet-geoman.css";

interface MapViewerProps {
    center: [number, number];
    zoom: number;
    geoJson: GeoJsonObject | null;
}

const MapViewer: React.FC<MapViewerProps> = ({ center, zoom , geoJson}) => {


    useEffect(() => {
        // Убеждаемся, что Leaflet использует правильные иконки
        delete (L as any).Icon.Default.prototype._getIconUrl;

        L.Icon.Default.mergeOptions({
            iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png').default,
            iconUrl: require('leaflet/dist/images/marker-icon.png').default,
            shadowUrl: require('leaflet/dist/images/marker-shadow.png').default,
        });
    }, []);

    function MapSettings() {
        const map = useMap();
        map.pm.addControls({
            position: 'topleft',
            drawCircleMarker: false,
            rotateMode: false,
        });
        return null;
    }
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
            <MapEditor geoJson={geoJson} />
            <MapSettings/>
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

        // Подключаем Leaflet-Geoman к карте
        map.pm.addControls({
            position: 'topleft',
            drawCircle: false,
            drawMarker: false,
            drawCircleMarker: false,
        });

        // Применяем стиль для GeoJSON
        // const geoJsonStyle = {
        //     color: '#ee5858',
        //     weight: 2,
        //     fillColor: '#a11ae0',
        //     fillOpacity: 0.5,
        // };
        //
        // // Добавляем GeoJSON слой на карту
        // if (geoJson) {
        //     L.geoJSON(geoJson, {
        //         style: geoJsonStyle,
        //     }).addTo(map);
        // }

        // Обработчики событий для геометрии
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
