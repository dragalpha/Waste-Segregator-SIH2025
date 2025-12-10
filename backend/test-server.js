// Simple test server to verify setup
const express = require('express');
const path = require('path');

const app = express();
app.use(express.static(path.join(__dirname, '../frontend')));

const PORT = 4000;
app.listen(PORT, () => {
  console.log(`✅ Test server running on http://localhost:${PORT}`);
  console.log(`✅ Frontend should be accessible now!`);
});

