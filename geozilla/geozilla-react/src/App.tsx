import React, { useState } from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import { Container, Box } from '@mui/material';
import MapViewer from './components/MapViewer';
import DataSender from "./components/DataSender";
import {GeoJsonObject} from "geojson";

const App = () => {
    const [geoJson, setGeoJson] = useState<GeoJsonObject | null>(null);

  return (
    <div className="App">
      <DataSender setGeoJson={setGeoJson}/>
        <Container>
            <Box marginTop={4} height="500px">
                <MapViewer geoJson={geoJson} center={[56.1322200, 47.2519400]} zoom={10} />
            </Box>
        </Container>
    </div>
  );
}

export default App;
