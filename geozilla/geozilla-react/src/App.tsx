import React, { useState } from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import { Container, Box, Button } from '@mui/material';
import FileUploader from './components/FileUploader';
import CoordinateInput from './components/CoordinateInput';
import MapViewer from './components/MapViewer';
import DataSender from "./components/DataSender";
import {GeoJsonObject} from "geojson";

const App = () => {
    const [editorOpen, setEditorOpen] = useState(false);
    const [geoJson, setGeoJson] = useState<GeoJsonObject | null>(null);

  return (
    <div className="App">
      <DataSender setGeoJson={setGeoJson}/>
        <Container>
            <Box marginTop={4} height="500px">
                <MapViewer geoJson={geoJson} center={[56.1322200, 47.2519400]} zoom={10} />
            </Box>
            <Button variant="contained" onClick={() => setEditorOpen(true)}>
                Open Zone Editor
            </Button>
        </Container>
    </div>
  );
}

export default App;
