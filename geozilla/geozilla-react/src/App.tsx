import React from 'react';
import './App.css';
import '@fontsource/roboto/300.css';
import '@fontsource/roboto/400.css';
import '@fontsource/roboto/500.css';
import '@fontsource/roboto/700.css';
import FileUploader from "./components/FileUploader";
import CoordinateInput from './components/CoordinateInput';
import {Box, Container} from "@mui/material";

function App() {
  return (
    <div className="App">
      <Container>
        <Box marginTop={4}>
          <FileUploader />
        </Box>
        <Box marginTop={4}>
          <CoordinateInput />
        </Box>
      </Container>
    </div>
  );
}

export default App;
