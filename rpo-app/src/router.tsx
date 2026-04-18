import { createBrowserRouter } from 'react-router';

import { FarFieldView } from './views/FarFieldView';
import { ProximityView } from './views/ProximityView';
import { SetupView } from './views/SetupView';
import { Shell } from './views/Shell';
import { ViewportShell } from './views/ViewportShell';

export const router = createBrowserRouter([
  {
    path: '/',
    Component: Shell,
    children: [{ index: true, Component: SetupView }],
  },
  {
    path: '/',
    Component: ViewportShell,
    children: [
      { path: 'far-field', Component: FarFieldView },
      { path: 'proximity', Component: ProximityView },
    ],
  },
]);
