import React, { useState } from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import MapViewer from './components/MapViewer';
import DataSender from "./components/DataSender";
import {FeatureCollection} from "geojson";

const App = () => {
    const [geoJson, setGeoJson] = useState<FeatureCollection | null>(null);

    console.log("### App")
  return (
    <div className="App">
        {!geoJson && <DataSender setGeoJson={setGeoJson}/>}
        {geoJson && <MapViewer geoJson={geoJson} setGeoJson={setGeoJson} center={[56.1322200, 47.2519400]} zoom={10} />}
    </div>
  );
}

export default App;
